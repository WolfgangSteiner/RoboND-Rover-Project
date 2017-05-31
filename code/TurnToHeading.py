import numpy as np
from utils import *

class TurnToHeading(object):
    def __init__(self, rover, heading):
        self.rover = rover
        self.heading = heading


    def heading_error(self):
        return normalize_angle_deg(self.heading - self.rover.yaw)


    def run(self):
        self.rover.throttle = 0
        if abs(self.rover.vel) > 0.2:
            self.rover.brake = 1
        else:
            d_theta = self.heading_error()
            abs_rate = min(15, max(1, abs(d_theta) / 4))
            sign_rate = np.sign(d_theta)
            self.rover.brake = 0
            self.rover.steer = abs_rate * sign_rate


    def is_heading_reached(self):
        return abs(self.heading_error()) < 1


    def next(self):
        if self.is_heading_reached():
            self.rover.steer = 0
            return None
        else:
            return self