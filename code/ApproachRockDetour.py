from StateQueue import StateQueue
from MoveDistance import MoveDistance
from TurnToBearing import TurnToBearing
from TurnToRock import TurnToRock
from perception import distance_to_rock
import math
from utils import *


class ApproachRockDetour(StateQueue):
    def __init__(self, rover, distance_to_rock):
        super().__init__(rover)
        phi = -20
        self.add(TurnToBearing(self.rover, phi))
        self.add(MoveDistance(self.rover, 2))
        self.add(TurnToRock(self.rover))