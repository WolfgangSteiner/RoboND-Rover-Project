import numpy as np
from utils import normalize_angle_deg

class Rotate(object):
    def __init__(self, rover, angle):
        self.rover = rover
        self.start_heading = self.rover.yaw
        self.end_heading = self.start_heading + angle
        self.total_angle = 0
        self.last_heading = self.rover.yaw
        self.half_heading = self.rover.yaw * 0.5 * angle
        self.angle = angle
        self.did_reach_half_angle = False

    def run(self):
        self.rover.brake = 0
        diff_angle = normalize_angle_deg(self.rover.yaw - self.last_heading)
        self.total_angle += diff_angle
        self.last_heading = self.rover.yaw

        self.rover.steer =  np.clip((self.angle - self.total_angle) / 4, 1, 15)

        #print("total angle: %.2f" % self.total_angle)
        #print("Rotate - rover.steer = %.2f" % self.rover.steer)


    def next(self):
        if self.total_angle >= self.angle:
            self.rover.steer = 0
            return None
        else:
            return self