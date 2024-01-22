import olympe
import os
import time
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy
from olympe.messages.ardrone3.PilotingState import (
    PositionChanged,
    AlertStateChanged,
    FlyingStateChanged,
    NavigateHomeStateChanged,
)

class FlightListener(olympe.EventListener):

    @olympe.listen_event(FlyingStateChanged() | AlertStateChanged() |
        NavigateHomeStateChanged())
    def onStateChanged(self, event, scheduler):
        print("{} = {}".format(event.message.name, event.args["state"]))

    @olympe.listen_event(PositionChanged())
    def onPositionChanged(self, event, scheduler):
        print(
            "latitude = {latitude} longitude = {longitude} altitude = {altitude}".format(
                **event.args
            )
        )

DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")

drone = olympe.Drone(DRONE_IP)
with FlightListener(drone):
    drone.connect()
    drone(
        FlyingStateChanged(state="hovering")
        | (TakeOff() & FlyingStateChanged(state="hovering"))
    ).wait()
    #drone(moveBy(10, 0, 0, 0)).wait()
    #olympe.Drone.streaming.start()
    drone(Landing()).wait()
    drone(FlyingStateChanged(state="landed")).wait()
    drone.disconnect()
