import numpy as np
from utils import *
from StopAndTurn import StopAndTurn
from perception import is_obstacle_ahead, is_overhanging_rock_ahead
from FindAndPickupRock import FindAndPickupRock
from SurveySpin import SurveySpin
from AvoidOverhangingRock import AvoidOverhangingRock



class MoveForward(object):
    def __init__(self, rover):
        self.rover = rover
        self.target_vel = 2.5
        self.throttle = 0.2
        self.last_survey_pos = [1e9, 1e9]
        self.a = 0.5


    def run(self):
        self.rover.brake = 0
        self.update_throttle()
        self.update_steer()


    def num_samples_found(self):
        return len(np.array(self.rover.samples_found).nonzero()[0])


    def next(self):
        if is_obstacle_ahead(self.rover):
            return StopAndTurn(self.rover)
        elif is_overhanging_rock_ahead(self.rover):
            return AvoidOverhangingRock(self.rover)
        elif self.is_rock_near():
            return FindAndPickupRock(self.rover)
        elif self.num_samples_found() == 6:
            print("RETURN HOME!")
            return None
        else:
            return self


    def update_throttle(self):
        if self.rover.vel < self.target_vel:
            self.rover.throttle = self.throttle
        else:
            self.rover.throttle = 0


    def update_steer(self):
        idx_nav_near = np.where((self.rover.nav_dists < 30) & (self.rover.nav_dists > 10))[0]
        if not idx_nav_near.size:
            max_nav_angle = 90
        else:
            max_nav_angle = np.max(self.rover.nav_angles[idx_nav_near] * 180/np.pi)

        idx_obs_near = np.where((self.rover.obs_dists < 20) & (self.rover.obs_dists > 0) & (self.rover.obs_angles > 0))[0]
        if not idx_obs_near.size:
            min_obs_angle = 90
        else:
            min_obs_angle = np.min(self.rover.obs_angles[idx_obs_near] * 180/np.pi)


        #print("Max nav angle: %.2f, Min obs angle: %.2f" % (max_nav_angle, min_obs_angle))

        new_steer = np.clip(min(max_nav_angle - 25,min_obs_angle - 35), -15, 15)
        self.rover.steer = self.a * new_steer + (1-self.a) * self.rover.steer


    def is_rock_near(self):
        return self.rover.rock_angles.size > 0
