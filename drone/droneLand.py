import olympe
import cv2
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing
from olympe.messages.camera import start_video_streaming, stop_video_streaming

def main():
    # Connect to the drone
    with olympe.Drone("192.168.42.1) as drone:
        # Set the camera mode to video streaming
        #drone(set_camera_mode(cam_id=0, value="streaming"))

        # Wait for the drone to be ready
        #drone.smart_sleep(5)

        # Take off
        drone(TakeOff()).wait()

        # Start video streaming
        #drone(start_video_streaming(cam_id=0))

        #try:
            # Fly straight for 3 seconds
            #drone(moveBy(0, 2, 0, 0)).wait()
            #drone.smart_sleep(3)

        #finally:
            # Stop video streaming
            #drone(stop_video_streaming(cam_id=0))

            # Land
            drone(Landing()).wait()

if __name__ == "__main__":
    main()
