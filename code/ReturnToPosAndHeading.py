from StateQueue import StateQueue
from math import atan2
import numpy as np
from TurnToHeading import TurnToHeading
from MoveDistance import MoveDistance
from Stop import Stop
from utils import distance, rad2deg


class ReturnToPosAndHeading(StateQueue):
    def __init__(self, rover, target_pos, target_heading):
        super().__init__(rover)
        self.target_pos = np.array(list(target_pos))
        self.target_heading = target_heading
        self.is_first_run = True


    def run(self):
        if self.is_first_run:
            diff = self.target_pos - np.array(self.rover.pos)
            heading_to_target = rad2deg(atan2(diff[1], diff[0])) + 180
            distance_to_target = distance(self.target_pos, self.rover.pos)
            self.add(Stop(self.rover))
            self.add(TurnToHeading(self.rover, heading_to_target))
            self.add(MoveDistance(self.rover, distance_to_target - 0.5, target_vel=-1))
            self.add(TurnToHeading(self.rover, self.target_heading))
            self.is_first_run = False
        else:
            super().run()

