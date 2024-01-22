import olympe
import os
import time

DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")


def test_physical_drone():
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    time.sleep(5)
    drone.disconnect()

if __name__ == "__main__":
    test_physical_drone()
