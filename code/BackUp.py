from Rotate import Rotate
import numpy as np
from utils import *

class BackUp(object):
    def __init__(self, rover):
        self.rover = rover
        self.did_move = False
        self.step_count = 0
        self.mode = ""
        self.did_move_step_count = 0
        self.is_done = False
        self.start_pos = np.array(rover.pos)


    def distance_moved(self):
        return distance(self.rover.pos, self.start_pos)


    def run(self):
        if self.is_done:
            return

        if not self.did_move and self.step_count > 40:
            self.rover.throttle *= -1


        if not self.did_move and self.distance_moved() > 0.5:
            self.did_move = True


        if abs(self.rover.roll) > 5:
            self.mode = "twist"
            self.rover.throttle = 0
            self.rover.steer = -15

        elif self.distance_moved() > 0.25:
            self.rover.steer = -15

        elif not self.did_move:
            self.rover.brake = 0
            self.rover.steer = 15
            self.rover.throttle = -1.0
            self.step_count +=1


    def next(self):
        if self.is_done:
            self.rover.throttle = 0
            self.rover.steer = 0
            return None

        elif self.did_move and self.distance_moved() > 0.5:
            self.rover.throttle = 0
            self.rover.brake = 1
            self.is_done = True
            return Rotate(self.rover, -22.5, fast=True)

        elif self.mode == 'twist' and abs(self.rover.roll) < 2.5:
            self.rover.steer = 0
            self.rover.throttle = 0
            self.is_done = True
            return Rotate(self.rover, -22.5, fast=True)


        else:
            return self
