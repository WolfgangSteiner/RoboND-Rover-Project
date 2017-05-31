from perception import bearing_to_rock_deg
from utils import *
from StateQueue import StateQueue
from Stop import Stop
from TurnToHeading import TurnToHeading
from ApproachRock import ApproachRock
from PickupRock import PickupRock
from ReturnToPosAndHeading import ReturnToPosAndHeading
from TurnToBearing import TurnToBearing

class FindAndPickupRock(StateQueue):
    def __init__(self, rover):
        super().__init__(rover)
        bearing = bearing_to_rock_deg(self.rover)
        self.add(Stop(self.rover))
        self.add(TurnToBearing(self.rover, bearing))
        self.add(ApproachRock(self.rover))
        self.add(Stop(self.rover))
        self.add(PickupRock(self.rover))
        if bearing < -5:
            self.add(ReturnToPosAndHeading(self.rover, self.rover.pos, self.rover.yaw))
        else:
            self.add(TurnToHeading(self.rover,self.rover.yaw))