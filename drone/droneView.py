
import time


 
    
import olympe
import os
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing

DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")


def test_moveby():
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    assert drone(TakeOff()).wait().success()
    time.sleep(2)
    drone(moveBy(0, 0,0 , 6)).wait()
    time.sleep(2)
    assert drone(Landing()).wait().success()
    time.sleep(5)
    drone.disconnect()


if __name__ == "__main__":
    test_moveby()
