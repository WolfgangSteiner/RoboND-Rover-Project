import numpy as np
import math


def deg2rad(phi):
    return np.pi * phi / 180

def rad2deg(phi):
    return 180 * phi / np.pi

def normalize_angle_deg(phi):
    phi_rad = deg2rad(phi)
    return rad2deg(np.arctan2(np.sin(phi_rad), np.cos(phi_rad)))

def distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))


def heading_to_pos_deg(p1, p2):
    diff = np.array(p2) - np.array(p1)
    return rad2deg(math.atan2(diff[1], diff[0]))
