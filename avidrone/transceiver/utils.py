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

    displacement = get_displacement(x_1, x_2, y_1, y_2, z_1, z_2)
    uclidean_distance = get_euclidean_distance(displacement)
    displacement_n = normalize_vector(displacement)
    distance = get_distance(displacement)
    theta = vector_angle(displacement_n, distance)
    direction = calculate_direction(theta)


    print(theta)
    return direction, distance


def get_displacement(x_1, x_2, y_1, y_2, z_1=0, z_2=0):
    displacement = [x_2 - x_1, y_2 - y_1, z_2 - z_1]
    return displacement


def get_distance(displacement):
    dx = displacement[0]
    dy = displacement[1]
    return (dx**2 + dy**2) ** -2


def get_euclidean_distance(displacement):
    dx = displacement[0]
    dy = displacement[1]
    dz = displacement[2]
    return (dx**2 + dy**2 + dz**2) ** -2


def normalize_vector(displacement):
    d_v = displacement
    normalized_v_d_array = d_v / np.linalg.norm(d_v)
    normalized_v_d = normalized_v_d_array.tolist()
    return normalized_v_d


def vector_angle(displacement, distance):
    fwd = [1, 0, 0]  # UAV's forward vector.
    a = np.dot(displacement, fwd)
    print(a / distance)
    theta = np.arccos(a / distance)

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
    uav_position = [10, 6, 2]
    beacon_position = [5, 1, 0]
    mock_direction_distance = mock_beacon(uav_position, beacon_position)
    print(mock_direction_distance)
