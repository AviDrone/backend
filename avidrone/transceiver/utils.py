import random

import numpy as np


def mock_beacon(uav_position, beacon_position):
    # UAV position (Currently random values)
    x_1 = uav_position[0]
    y_1 = uav_position[1]
    z_1 = uav_position[2]

    # Beacon position (Currently random values)
    x_2 = beacon_position[0]
    y_2 = beacon_position[1]
    z_2 = beacon_position[2]

    displacement_vector = get_displacement(x_1, x_2, y_1, y_2, z_1, z_2)
    uclidean_distance = get_uclidean_distance(displacement_vector)
    displacement_normal = normalize_vector(displacement_vector)
    theta = vector_angle(displacement_normal, uclidean_distance)
    direction = calculate_direction(theta)
    distance = uclidean_distance[0]

    return direction, distance


def get_displacement(x_1, x_2, y_1, y_2, z_1=0, z_2=0):
    dx = x_2 - x_1
    dy = y_2 - y_1
    dz = z_2 - z_1
    displacement = [dx, dy, dz]
    return displacement


def get_uclidean_distance(displacement):
    dx = displacement[0]
    dy = displacement[1]
    dz = displacement[2]

    d = (dx**2 + dy**2 + dz**2) ** -2
    d_xy = (dx**2 + dy**2) ** -2
    return d, d_xy


def normalize_vector(displacement_vector):
    d_v = displacement_vector
    normalized_v_d_array = d_v / np.linalg.norm(d_v)
    normalized_v_d = normalized_v_d_array.tolist()
    return normalized_v_d


def vector_angle(d_v, euclidean_distance):
    fwd = [1, 0, 0]  # UAV's forward vector.
    d_v_cross_fwd = np.dot(d_v, fwd)
    theta = np.arccos(d_v_cross_fwd) / euclidean_distance[1]

    # Random value to account for measurement inconsistencies.
    # Using -15 and 15 makes it likely that the beacon gets the
    # wrong direction roughly a third of the time.

    theta = theta + random.uniform(-15, 15)
    return theta


def calculate_direction(theta):
    """
    led 0: Theta -90 to -30
    led 1: Theta -30 to -10
    led 2: Theta -10 to +10
    led 3: Theta +10 to +30
    led 4: Theta +30 to +90

    """
    direction = -1  # Initial, unknown direction.

    if -90 <= theta < -30:
        direction = 0
        print("led 0")

    if -30 <= theta < -10:
        direction = 1
        print("led 1")

    if -10 <= theta < 10:
        direction = 2
        print("led 2")

    if 10 <= theta < 30:
        direction = 3
        print("led 3")

    if 30 <= theta <= 90:
        direction = 4
        print("led 4")
    else:
        print("invalid")
    return direction


if __name__ == "__main__":
    uav_position = [2, 2, 5]
    beacon_position = [2, 2, 1]
    mock_beacon(uav_position, beacon_position)
    print(mock_beacon)
