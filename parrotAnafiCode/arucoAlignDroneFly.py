# not used but provided by the example??
import shlex
import subprocess
import tempfile

import cv2
import cv2.aruco as aruco


from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.Piloting import moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.PilotingSettingsState import MaxTiltChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
import olympe.messages.gimbal as gimbal


# these libraries are used
import math
import numpy as np
import csv
import os
import queue
import threading
import time
import uuid # self added
import olympe
from olympe.video.renderer import PdrawRenderer



olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1") # this is the physical drone IP address
DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT")

class StreamingExample:

    def __init__(self):
        # Create the olympe.Drone object from its IP address
        self.drone = olympe.Drone(DRONE_IP)

        # this creates a folder with a unique name in a known directory location
        unique_filename = str(uuid.uuid4())
        self.recorded_video = "/home/crk/Documents/" + unique_filename
        os.mkdir(self.recorded_video)
        print(f"Olympe streaming example output dir: {self.recorded_video}")
        
        # stats
        self.h264_frame_stats = []
        self.h264_stats_file = open(os.path.join(self.recorded_video, "h264_stats.csv"), "w+")
        self.h264_stats_writer = csv.DictWriter(
            self.h264_stats_file, ["fps", "bitrate"]
        )
        self.h264_stats_writer.writeheader()
        self.frame_queue = queue.Queue()
        self.processing_thread = threading.Thread(target=self.yuv_frame_processing)
        self.renderer = None
        

    def start(self):
        # Connect to drone
        assert self.drone.connect(retry=3)

        if DRONE_RTSP_PORT is not None:
            self.drone.streaming.server_addr = f"{DRONE_IP}:{DRONE_RTSP_PORT}"

        # You can record the video stream from the drone if you plan to do some
        # post processing.
        self.drone.streaming.set_output_files(
            video=os.path.join(self.recorded_video, "streaming.mp4"),
            metadata=os.path.join(self.recorded_video, "streaming_metadata.json"),
        )

        # Setup your callback functions to do some live video processing
        self.drone.streaming.set_callbacks(
            raw_cb=self.yuv_frame_cb,
            h264_cb=self.h264_frame_cb,
            start_cb=self.start_cb,
            end_cb=self.end_cb,
            flush_raw_cb=self.flush_cb,
        )
        # Start video streaming
        self.drone.streaming.start()
        self.renderer = PdrawRenderer(pdraw=self.drone.streaming)
        self.running = True
        self.processing_thread.start()

    def stop(self):
        self.running = False
        self.processing_thread.join()
        if self.renderer is not None:
            self.renderer.stop()
        # Properly stop the video stream and disconnect
        assert self.drone.streaming.stop()
        assert self.drone.disconnect()
        self.h264_stats_file.close()

    def yuv_frame_cb(self, yuv_frame):

        yuv_frame.ref()
        self.frame_queue.put_nowait(yuv_frame)


    def yuv_frame_processing(self):

        # variables for image processing
        k = np.array([[996.80114623, 0., 690.13286978],
                      [0., 961.38301889, 361.68868703],
                      [0., 0., 1.]])
        
        d = np.array([0.01849482, 0.01653952, -0.0063708, 0.0083632, -0.21739897])

        while self.running:
            try:
                yuv_frame = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue
 
            yuv_2d_array = yuv_frame.as_ndarray()
            # this grabs the frame array from the frame object

            cv2_cvt_color_flag = {
            olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,
            }[yuv_frame.format()]
            # this creates the flag that is passed into the following lines that is used for bgr conversion

            bgr_frame = cv2.cvtColor(yuv_2d_array, cv2_cvt_color_flag)
            gray_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2GRAY)
            # yuv > bgr > gray

            dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50) # the aruco ID must be below 50
            parameters = aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray_frame, dictionary, parameters=parameters)

            if len(corners) > 0:
                    #print("ArUco marker detected, rotating the drone.")
                    #self.rotate_drone(90)  # Rotate the drone by 90 degrees, works 
                    for i in range(0, len(ids)):

                        # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                        rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.02, k, d)

                        rmat, _= cv2.Rodrigues(rvec)
                        #rmat2= rmat[0][0] 

                        angle_z = np.arccos(rmat[0, 0])  # Cosine inverse of the first element
                        # Convert angle from radians to degrees for easier interpretation
                        angle_z_degrees = np.degrees(angle_z)

                        # Evaluate the complexity of the rotation by examining the off-diagonal elements
                        off_diagonal_sum = np.sum(np.abs([rmat[0, 1], rmat[0, 2], rmat[1, 0], rmat[1, 2], rmat[2, 0], rmat[2, 1]]))
                        # Classification based on the angle and the sum of off-diagonal elements
                        print("Angle Z: ", angle_z_degrees)
                        print("off_Sum: ", off_diagonal_sum)

                    if angle_z_degrees < 25 and off_diagonal_sum < 1:
                        assert self.drone(moveBy(0,0,0,0) >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
                        self.droneSuccess()
                        time.sleep(1)
                        print("Landed!!!")                    
                    else:
                        #print("drone turn left")
                        assert self.drone(moveBy(0,0,0, -.1) >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
                    #if (rmat2[0] > 0.97 and rmat2[0] < .99):
                        #assert self.drone(moveBy(0,0,0,0) >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
                        #self.droneSuccess()
                        #time.sleep(1)
                      #  print("Landed!!!")

                    #if rmat2[1] > 0:
                    #    print("drone turn left")
                    #    assert self.drone(moveBy(0,0,0,.1) >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()

                    
                    #if rmat2[1] < 0:
                    #    print("drone turn right")
                    #    assert self.drone(moveBy(0,0,0,-.1) >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()

                   # else:
                        #assert self.drone(moveBy(0,0,0,.1) >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
                     #   print("turn right")

            

                   # print("rvec X:\n", rmat) # this is the rotation vector that can be turned into a rotational matrix using cv2.rodrigues
                   ## print("rvec X:\n", rmat2[0]) # this is the rotation vector that can be turned into a rotational matrix using cv2.rodrigues
                   # print("rvec Y:\n", rmat2[1]) # this is the rotation vector that can be turned into a rotational matrix using cv2.rodrigues
                   # print("rvec Y:\n", rmat2[2]) # this is the rotation vector that can be turned into a rotational matrix using cv2.rodrigues

                    #print("rvec X:\n", r1) # this is the rotation vector that can be turned into a rotational matrix using cv2.rodrigues

                    #print("\n\ntvec:\n",2*tvec) # NOTE: I am not sure why, but you need to double the tvec values for them to be accurate
                    
            yuv_frame.unref()
       
    def rotate_drone(self, angle):
        """
        Rotate the drone by a specified angle in degrees.
        """
        assert self.drone(moveBy(0, 0, 0, np.radians(angle)) >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
    
    def flush_cb(self, stream):
        if stream["vdef_format"] != olympe.VDEF_I420:
            return True
        while not self.frame_queue.empty():
            self.frame_queue.get_nowait().unref()
        return True

    def start_cb(self):
        pass

    def end_cb(self):
        pass

    def h264_frame_cb(self, h264_frame):

        # Get a ctypes pointer and size for this h264 frame
        frame_pointer, frame_size = h264_frame.as_ctypes_pointer()

        # Compute some stats and dump them in a csv file
        info = h264_frame.info()
        frame_ts = info["ntp_raw_timestamp"]
        if not bool(info["is_sync"]):
            while len(self.h264_frame_stats) > 0:
                start_ts, _ = self.h264_frame_stats[0]
                if (start_ts + 1e6) < frame_ts:
                    self.h264_frame_stats.pop(0)
                else:
                    break
            self.h264_frame_stats.append((frame_ts, frame_size))
            h264_fps = len(self.h264_frame_stats)
            h264_bitrate = 8 * sum(map(lambda t: t[1], self.h264_frame_stats))
            self.h264_stats_writer.writerow({"fps": h264_fps, "bitrate": h264_bitrate})

    def fly(self):
        # Takeoff, fly, land, ...
        # to understand this section, reference the following page
        # https://developer.parrot.com/docs/olympe/userguide/basics/moving_around.html
        assert self.drone.connect(retry=3)  

        if DRONE_RTSP_PORT is not None:
            self.drone.streaming.server_addr = f"{DRONE_IP}:{DRONE_RTSP_PORT}"

        assert self.drone(TakeOff()>> FlyingStateChanged(state="hovering", _timeout=10)).wait().success()
        assert self.drone(moveBy(0, 0, 0, 0)>> FlyingStateChanged(state="hovering", _timeout=10)).wait().success()

    def land(self):
        # Takeoff, fly, land, ...
        # to understand this section, reference the following page
        # https://developer.parrot.com/docs/olympe/userguide/basics/moving_around.html
        assert self.drone.connect(retry=3)

        if DRONE_RTSP_PORT is not None:
            self.drone.streaming.server_addr = f"{DRONE_IP}:{DRONE_RTSP_PORT}"

        assert self.drone(Landing()).wait().success()

    def move_gimbal(self,attitude):
        self.drone(
            gimbal.set_target(
                gimbal_id = 0,
                control_mode = "position",
                yaw_frame_of_reference = "absolute",
                yaw = 0.0,
                pitch_frame_of_reference = "absolute",
                pitch = attitude,
                roll_frame_of_reference = "absolute",
                roll = 0.0
            )
        ).wait()

    # Set maximum gimbal speed for the pitch axis
    def max_gimbal_speed(self,pitch_speed):
        self.drone(
        gimbal.set_max_speed(
            gimbal_id=0,
            yaw=0.0,  # Assuming no change needed for yaw
            pitch=pitch_speed,  # Set your desired pitch speed here
            roll=0.0  # Assuming no change needed for roll
            )
        ).wait()

    def droneSuccess(self): 
        self.drone.land()
        self.drone.stop()


def test_streaming():
    drone = StreamingExample()
    
    # Start the video stream
    drone.fly() 
    time.sleep(1)

    

    drone.move_gimbal(0)
    time.sleep(2)

    drone.move_gimbal(-100)
    time.sleep(2)


    drone.start()
    time.sleep(30)

    drone.land()
    
    drone.stop()



if __name__ == "__main__":
    test_streaming()
