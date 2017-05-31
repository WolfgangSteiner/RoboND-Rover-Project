from utils import distance
from Stop import Stop
import numpy as np

class MoveDistance(object):
    def __init__(self, rover, dist, target_vel=2):
        self.rover = rover
        self.dist = dist
        self.target_vel = target_vel
        self.throttle = 0.2 * np.sign(target_vel)
        self.is_done = False
        self.is_first_run = True


    def run(self):
        self.rover.steer = 0
        if self.is_first_run:
            self.start_pos = self.rover.pos
            self.is_first_run = False


        if not self.is_done and distance(self.rover.pos, self.start_pos) >= self.dist:
            self.is_done = True


        if self.is_done:
            self.rover.brake = 1
            self.rover.throttle = 0
        elif abs(self.rover.vel) < abs(self.target_vel):
            self.rover.brake = 0
            self.rover.throttle = self.throttle
        else:
            self.rover.throttle = 0


    def next(self):
        if self.is_done and abs(self.rover.vel) < 0.2:
            return None
        else:
            return self
