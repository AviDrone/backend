import math
import random

import numpy as np

random.seed(492)
# distances are euclidean


def mock_beacon(uav_pos, beacon_pos):
    # UAV position
    x_1 = uav_pos[0]
    y_1 = uav_pos[1]
    z_1 = uav_pos[2]

    # Beacon position
    x_2 = beacon_pos[0]
    y_2 = beacon_pos[1]
    z_2 = beacon_pos[2]

    displacement = get_displacement(x_1, x_2, y_1, y_2, z_1, z_2)
    euclidean_dist = get_distance_xyz(displacement)
    disp_n = normalize(displacement)
    dist = get_distance_xy(displacement)
    theta = get_angle(disp_n)
    direction = get_direction(theta)
    return direction, dist


def get_displacement(x_1, x_2, y_1, y_2, z_1=0, z_2=0):
    return [x_2 - x_1, y_2 - y_1, z_2 - z_1]


def get_distance_xy(disp):
    return (disp[0] ** 2 + disp[1] ** 2) ** -2


def get_distance_xyz(disp):
    return (disp[0] ** 2 + disp[1] ** 2 + disp[2] ** 2) ** -2


def normalize(disp):
    d_v = disp / np.linalg.norm(disp)
    d_v_normal = d_v.tolist()
    return d_v_normal


def get_angle(disp):
    fwd = [1, 0]  # UAV's forward vector.
    v_d = [disp[0], disp[1]]
    d_xy = get_distance_xy(disp)
    theta = np.arccos(np.dot(v_d, fwd) / d_xy)

    # To account for measurement inconsistencies. We use a random value
    # between -15 and 15. That makes it likely that the beacon gets the
    # wrong direction roughly a third of the time.

    # print(theta + random.uniform(-15, 15))

    return theta + random.uniform(-15, 15)


def get_direction(theta):
    """
    led 0: Theta -90 to -30
    led 1: Theta -30 to -10
    led 2: Theta -10 to +10
    led 3: Theta +10 to +30
    led 4: Theta +30 to +90

    """

    direction = -1  # direction not acquired

    if -90 <= theta < -30:
        direction = 0

    if -30 <= theta < -10:
        direction = 1

    if -10 <= theta < 10:
        direction = 2

    if 10 <= theta < 30:
        direction = 3

    if 30 <= theta <= 90:
        direction = 4

    return direction


if __name__ == "__main__":
    uav_position = [12, 120, 25]  # Example
    beacon_position = [60, 60, 1]  # Example
    mock_beacon_ = mock_beacon(uav_position, beacon_position)
    print("Direction, Distance: ", mock_beacon_)
