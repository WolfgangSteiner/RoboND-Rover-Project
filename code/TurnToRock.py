from perception import is_rock_near, bearing_to_rock
from utils import *
import numpy as np


class TurnToRock(object):
    def __init__(self, rover):
        self.rover = rover
        self.default_bearing = 90


    def target_bearing(self):
        if is_rock_near(self.rover):
            bearing = rad2deg(bearing_to_rock(self.rover))
            self.default_bearing = 90 * np.sign(bearing)
            return bearing
        else:
            return self.default_bearing


    def run(self):
        if abs(self.rover.vel) > 0.2:
            self.rover.brake = 1

        else:
            self.rover.brake = 0
            print("target bearing: ", self.target_bearing())
            bearing = self.target_bearing()
            min_steer = 1 * np.sign(bearing)
            max_steer = 5 * np.sign(bearing)
            min_steer, max_steer = min(min_steer, max_steer), max(min_steer, max_steer)
            self.rover.steer = np.clip(self.target_bearing() // 4, min_steer, max_steer)


    def next(self):
        if abs(self.target_bearing()) < 2.5:
            self.rover.steer = 0
            return None
        else:
            return self