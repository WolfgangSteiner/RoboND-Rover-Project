import numpy as np
import cv2
from utils import deg2rad, rad2deg


# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_threshold(img, min, max):
    return cv2.inRange(img, np.array(min), np.array(max))


# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


def calc_range(xpix, ypix):
    return np.sqrt(xpix**2 + ypix**2)


def calc_angle(xpix, ypix):
    return np.arctan2(ypix, xpix)


def impose_range(xpix, ypix, range=40):
    dist = calc_range(xpix,ypix)
    return xpix[dist < range], \
           ypix[dist < range]


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    return calc_range(x_pixel, y_pixel), \
           calc_angle(x_pixel, y_pixel)


# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    yaw *= np.pi / 180.0
    return np.cos(yaw) * xpix - np.sin(yaw) * ypix, \
           np.sin(yaw) * xpix + np.cos(yaw) * ypix


# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    return xpix_rot * scale + xpos, \
           ypix_rot * scale + ypos


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(Rover, xpix, ypix, scale=0.1):
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, Rover.yaw)
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, Rover.pos[0], Rover.pos[1], scale)
    # Perform rotation, translation and clipping all at once
    world_size = 200
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world


# Define a function to perform a perspective transform
def perspective_transform(img):
    h,w = img.shape[0:2]
    if perspective_transform.M is None:
        src = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
        dst_size = 5
        dst_offset = 6
        dst = np.float32([
            [w//2 - dst_size, h - dst_offset],
            [w//2 + dst_size, h - dst_offset],
            [w//2 + dst_size, h - 2 * dst_size - dst_offset],
            [w//2 - dst_size, h - 2 * dst_size - dst_offset],
        ])

        perspective_transform.M = cv2.getPerspectiveTransform(src, dst)

    return cv2.warpPerspective(img, perspective_transform.M, (w, h))

perspective_transform.M = None


def process_layer(Rover, mask, range=50):
    xpix,ypix = rover_coords(mask)
    xpix,ypix = impose_range(xpix,ypix,range=range)
    return pix_to_world(Rover, xpix, ypix), (xpix, ypix)


def check_angle(angle, max_angle):
    return angle < max_angle or angle > 360 - max_angle


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):

    # Perform perception steps to update Rover()
    # TODO:
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    # 2) Apply perspective transform
    warped_img = perspective_transform(Rover.img)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    obs_mask = color_threshold(Rover.img, (0,0,0), (116,116,116))
    nav_mask = color_threshold(Rover.img, (160,160,160), (255,255,255))
    rock_mask = color_threshold(Rover.img, (150,140, 0), (255,255,32))

    obs_mask = perspective_transform(obs_mask)
    nav_mask = perspective_transform(nav_mask)
    rock_mask = perspective_transform(rock_mask)

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = obs_mask
    Rover.vision_image[:,:,1] = rock_mask
    Rover.vision_image[:,:,2] = nav_mask



    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates
    (obs_x_world, obs_y_world), (obs_x_rover, obs_y_rover)= process_layer(Rover, obs_mask)
    (rock_x_world, rock_y_world), (rock_x_rover, rock_y_rover)= process_layer(Rover, rock_mask)
    (nav_x_world, nav_y_world), (nav_x_rover, nav_y_rover) = process_layer(Rover, nav_mask)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
    if check_angle(Rover.pitch, 0.5) and check_angle(Rover.roll, 0.5):
        Rover.worldmap[obs_y_world, obs_x_world, 0] = 255
        Rover.worldmap[rock_y_world, rock_x_world, 1] = 255
        Rover.worldmap[nav_y_world, nav_x_world, 2] = 255



    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(nav_x_rover, nav_y_rover)
    Rover.obs_dists, Rover.obs_angles = to_polar_coords(obs_x_rover, obs_y_rover)
    Rover.rock_dists, Rover.rock_angles = to_polar_coords(rock_x_rover, rock_y_rover)

    return Rover


def is_overhanging_rock_ahead(Rover):
    h,w = Rover.img.shape[0:2]
    black_rock_mask = cv2.inRange(Rover.img, (0,0,0), (8,8,8))
    x_margin = w//3
    x1,x2 = x_margin, w - x_margin

    black_rock_idx_y, black_rock_idx_x = black_rock_mask[:,x1:x2].nonzero()
    if black_rock_idx_y.size == 0:
        return False

    return np.min(black_rock_idx_y) < 4 and np.max(black_rock_idx_y) > h * 0.575



def is_obstacle_ahead(Rover, range=20, bearing=0):
    idx_in_front = np.where((np.abs(Rover.obs_angles - bearing) < deg2rad(15)) & (Rover.obs_dists < range))[0]

    if len(idx_in_front) < 5:
        print("Mindist: N/A")
        return False
    else:
        min_dist = np.min(Rover.obs_dists[idx_in_front])
        print("Mindist: %.2f" % min_dist)
        return True


def distance_to_obstacle(Rover, bearing=0):
    idx_in_front = np.where((np.abs(Rover.obs_angles - bearing) < deg2rad(15)))[0]

    if len(idx_in_front) < 5:
        return 1000
    else:
        return np.min(Rover.obs_dists[idx_in_front])



def is_rock_ahead(Rover):
    idx_in_front = np.where(np.abs(Rover.rock_angles) < deg2rad(2.5))[0]
    return len(idx_in_front)


def is_rock_near(Rover):
    return Rover.rock_angles.size > 0


def bearing_to_rock(Rover):
    return np.mean(Rover.rock_angles)


def bearing_to_rock_deg(Rover):
    return rad2deg(bearing_to_rock(Rover))



def distance_to_rock(Rover):
    if len(Rover.rock_dists):
        return np.min(Rover.rock_dists)
    else:
        return 1e9



def obstacle_bearing(Rover, direction, min_dist=0, max_dist=30):
    if direction > 0:
        idx_obs_near = np.where((Rover.obs_dists < max_dist) & (Rover.obs_dists > min_dist) & (Rover.obs_angles > 0))[0]
    else:
        idx_obs_near = np.where((Rover.obs_dists < max_dist) & (Rover.obs_dists > min_dist) & (Rover.obs_angles < 0))[0]

    if not idx_obs_near.size:
        return 90 * np.sign(direction)
    else:
        return rad2deg(np.min(Rover.obs_angles[idx_obs_near]))

