from StateQueue import StateQueue
from MoveDistance import MoveDistance
from TurnToBearing import TurnToBearing
from TurnToHeading import TurnToHeading
from Rotate import Rotate


class AvoidOverhangingRock(StateQueue):
    def __init__(self, rover):
        super().__init__(rover)
        self.heading = self.rover.yaw

        self.add(MoveDistance(rover, 1, -1))
        self.add(Rotate(rover, -45, fast=True))
        self.add(MoveDistance(rover, 2, 1))
        self.add(TurnToHeading(rover, self.heading))
