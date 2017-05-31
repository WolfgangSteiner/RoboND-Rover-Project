from utils import *
from PickupRock import *
from perception import obstacle_bearing, distance_to_rock, distance_to_obstacle
from ApproachRockDetour import ApproachRockDetour


class ApproachRock(object):
    def __init__(self, rover):
        self.rover = rover
        self.target_vel = 0.5
        self.throttle = 0.1
        self.mode = "first approach"
        self.distance_to_rock = distance_to_rock(rover) / 10.0


    def run(self):
        self.rover.brake = 0
        self.update_steering_approach_rock()
        self.update_throttle()
        print("distance to rock: ", distance_to_rock(self.rover))


    def next(self):
        if self.rover.near_sample:
            return None
        elif self.mode == "first approach":#and self.is_rock_near_obstacle():
            self.mode = "final approach"
            return ApproachRockDetour(self.rover, self.distance_to_rock)
        else:
            return self


    def is_rock_near_obstacle(self):
        return abs(distance_to_rock(self.rover) - distance_to_obstacle(self.rover)) < 0.5



    def update_steering_approach_rock(self):
        if self.rover.rock_angles.size == 0:
            bearing = 0.0
        else:
            bearing = rad2deg(np.mean(self.rover.rock_angles))

        if False:
            obs_bearing = obstacle_bearing(self.rover, bearing)

            obs_offset = 30

            if bearing > 0:
                steer_angle = min(bearing, obs_bearing - obs_offset)
            else:
                steer_angle = max(bearing, obs_bearing + obs_offset)
            print("obs_bearing: ",  obs_bearing)

        self.rover.steer = bearing
        print("self.rover.steer: ", self.rover.steer)


    def update_throttle(self):
        if self.rover.vel < self.target_vel:
            self.rover.throttle = self.throttle
        else:
            self.rover.throttle = 0
