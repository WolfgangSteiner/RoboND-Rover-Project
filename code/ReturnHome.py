from perception import is_obstacle_ahead
from utils import *
from StopAndTurn import StopAndTurn
from TurnToHeading import TurnToHeading
from StateQueue import StateQueue
from TurnToFreeBearing import TurnToFreeBearing


class ReturnHome(object):
    def __init__(self, rover):
        self.rover = rover


    def distance_to_target(self):
        return distance(self.rover.pos, self.rover.home_pos)


    def target_velocity(self):
        d = self.distance_to_target()
        if d > 10.0:
            return 4.0
        else:
            return 1.0


    def throttle(self):
        if self.distance_to_target() > 10.0:
            return 0.4
        else:
            return 0.2


    def run(self):
        print("target heading:", self.target_heading())
        print("target bearing:", self.target_bearing())

        self.update_throttle()
        self.update_steering()


    def next(self):
        if is_obstacle_ahead(self.rover):
            return StopAndTurn(self.rover)
        elif abs(self.target_bearing()) > 180 - 45:
            sq = StateQueue(self.rover)
            sq.add(TurnToHeading(self.rover, self.target_heading()))
            sq.add(TurnToFreeBearing(self.rover))
            return sq
        elif self.distance_to_target() < 0.25:
            self.rover.brake = 1
            self.rover.throttle = 0
            return None
        else:
            return self


    def target_heading(self):
        return heading_to_pos_deg(self.rover.pos, self.rover.home_pos)


    def target_bearing(self):
        return normalize_angle_deg(self.target_heading() - self.rover.yaw)


    def update_steering(self):
        min_nav_angle = rad2deg(np.min(self.rover.nav_angles)) + 45
        max_nav_angle = rad2deg(np.max(self.rover.nav_angles)) - 45

        idx_obs_max = np.where((self.rover.obs_dists < 20) & (self.rover.obs_dists > 0) & (self.rover.obs_angles > 0))[
            0]
        if not idx_obs_max.size:
            max_obs_angle = 90
        else:
            max_obs_angle = np.min(self.rover.obs_angles[idx_obs_max] * 180 / np.pi)


        idx_obs_min = np.where((self.rover.obs_dists < 20) & (self.rover.obs_dists > 0) & (self.rover.obs_angles < 0))[
            0]
        if not idx_obs_min.size:
            min_obs_angle = -90
        else:
            min_obs_angle = np.max(self.rover.obs_angles[idx_obs_min] * 180 / np.pi)


        min_angle = max(min_nav_angle, min_obs_angle)
        max_angle = min(max_nav_angle, max_obs_angle)

        self.rover.steer = np.clip(self.target_bearing(), min_angle, max_angle)


    def update_throttle(self):
        if self.rover.vel < self.target_velocity():
            self.rover.throttle = self.throttle()
        else:
            self.rover.throttle = 0


