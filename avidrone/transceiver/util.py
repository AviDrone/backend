import math
import random

import numpy as np


def mock_beacon(uav_pos, beacon_pos):
    # UAV position
    x_1 = uav_pos[0]
    y_1 = uav_pos[1]
    z_1 = uav_pos[2]

    # Beacon position
    x_2 = beacon_pos[0]
    y_2 = beacon_pos[1]
    z_2 = beacon_pos[2]

    disp = get_displacement(x_1, x_2, y_1, y_2, z_1, z_2)
    distance = get_euclidean_distance(disp)
    disp_n = normalize_vector(disp)
    dist = get_distance(disp)
    theta = vector_angle(disp_n)
    direction = calculate_direction(theta)

    return direction, distance


def get_displacement(x_1, x_2, y_1, y_2, z_1=0, z_2=0):
    return [x_2 - x_1, y_2 - y_1, z_2 - z_1]


def get_distance(disp):
    return (disp[0] ** 2 + disp[1] ** 2) ** -2


def get_euclidean_distance(disp):
    return (disp[0] ** 2 + disp[1] ** 2 + disp[2] ** 2) ** -2


def normalize_vector(disp):
    d_v = disp / np.linalg.norm(disp)
    d_v_normal = d_v.tolist()
    return d_v_normal


def vector_angle(disp):
    fwd = [1, 0]  # UAV's forward vector.
    v_d = [disp[0], disp[1]]
    d_xy = get_distance(disp)
    theta = np.arccos(np.dot(v_d, fwd) / d_xy)

    # To account for measurement inconsistencies. We use a random value between
    # -15 and 15. That makes it likely that the beacon gets the wrong direction
    # roughly a third of the time.

    return theta + random.uniform(-15, 15)


def calculate_direction(theta):
    """
    led 0: Theta -90 to -30
    led 1: Theta -30 to -10
    led 2: Theta -10 to +10
    led 3: Theta +10 to +30
    led 4: Theta +30 to +90

    """

    led_dir = -1

    if -90 <= theta < -30:
        led_dir = 0

    if -30 <= theta < -10:
        led_dir = 1

    if -10 <= theta < 10:
        led_dir = 2

    if 10 <= theta < 30:
        led_dir = 3

    if 30 <= theta <= 90:
        led_dir = 4

    return led_dir


if __name__ == "__main__":
    uav_position = [12, 12, 0]  # Example
    beacon_position = [6, 6, 0]  # Example
    mock_beacon_ = mock_beacon(uav_position, beacon_position)
    print("Direction, Distance: ", mock_beacon_)
