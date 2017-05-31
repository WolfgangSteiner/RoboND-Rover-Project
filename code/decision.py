import numpy as np
from utils import deg2rad, rad2deg, normalize_angle_deg
from BackUp import BackUp
import random



def dist_to_rock(Rover):
    idx_in_front = np.where(np.abs(Rover.obs_angles) < deg2rad(2.5))[0]

    if not len(idx_in_front):
        print("Dist to rock: N/A")
        return None

    min_dist = np.min(Rover.rock_dists[idx_in_front])
    print("Dist to rock: %.2f" % min_dist)
    return min_dist






def update_steering(Rover):
    Rover.steer = calc_steering_angle(Rover)



def update_throttle(Rover):
    if Rover.objective == 'mapping':
        target_velocity = Rover.max_vel
        throttle = Rover.throttle_set
    else:
        target_velocity = 0.5
        throttle = 0.1

    if Rover.vel < target_velocity:
        Rover.throttle = throttle
    else:
        Rover.throttle = 0


def stop_rover(Rover):
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.steer = 0
    Rover.mode = 'stop'


def turn_rover(Rover, rate=-15):
    Rover.throttle = 0
    Rover.brake = 0
    Rover.steer = rate # Could be more clever here about which way to turn


def decision_step(Rover):
    if Rover.home_pos is None:
        assert(Rover.pos is not None)
        Rover.home_pos = Rover.pos


    if Rover.stuck_counter > 100 and not type(Rover.state_machine.current_state()) == type(BackUp) :
        Rover.state_machine.push_front(BackUp(Rover))
        Rover.stuck_counter = 0

    Rover.state_machine.run()

    if abs(Rover.throttle) > 0:
        if abs(Rover.vel) < 0.2:
            Rover.stuck_counter += 1
        else:
            Rover.stuck_counter = 0

    return Rover

