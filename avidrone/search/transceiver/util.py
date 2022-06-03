#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    TRANSCEIVER UTIL
"""

# For interfacing with the C code
import logging
import math
import random

import numpy as np

# logging
log = logging.getLogger(__name__)
# Set to logging.DEBUG to see individual parameter values
log.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s  [%(levelname)s]  %(message)s")
file_handler = logging.FileHandler("util.log")
file_handler.setFormatter(formatter)
log.addHandler(file_handler)


def get_displacement(x_1, x_2, y_1, y_2, z_1=0, z_2=0):
    displacement = [x_2 - x_1, y_2 - y_1, z_2 - z_1]
    log.debug(f"displacement: {displacement}")
    return displacement


def get_distance_xy(disp):
    distance_xy = math.sqrt((disp[0] ** 2 + disp[1] ** 2))
    log.debug(f"distance_xy: {distance_xy}")
    return distance_xy


def get_distance_xyz(disp):
    distance_xyz = math.sqrt((disp[0] ** 2 + disp[1] ** 2 + disp[2] ** 2))
    log.debug(f"distance_xyz: {distance_xyz}")
    return distance_xyz


def normalize(disp):
    d_v = disp / np.linalg.norm(disp)
    d_v_normal = d_v.tolist()
    log.debug(f"d_v_normal: {d_v_normal}")
    return d_v_normal


def get_theta(disp):
    fwd = [1, 0]  # UAV's forward vector.
    v_d = [disp[0], disp[1]]
    d_xy = get_distance_xy(disp)

    if d_xy != 0:
        theta = np.arccos(np.dot(v_d, fwd) / d_xy)

    else:
        d_xy = 0.001  # to avoid division by 0
        theta = np.arccos(np.dot(v_d, fwd) / d_xy)

    # To account for measurement inconsistencies. We use a random value
    # between -15 and 15. That makes it likely that the beacon gets the
    # wrong direction roughly a third of the time.

    theta_random = theta + random.uniform(-15, 15)
    log.debug(
        f"theta + random.uniform(-15, 15): {theta + random.uniform(-15, 15)}")
    return theta_random


def get_direction(theta):
    """
    led 0: Theta -90 to -30
    led 1: Theta -30 to -10
    led 2: Theta -10 to +10
    led 3: Theta +10 to +30
    led 4: Theta +30 to +90

    """

    direction = None  # direction not acquired

    if theta < -30:
        log.debug("Led 0")
        log.info(direction)
        direction = 0

    elif -30 <= theta < -10:
        direction = 1

    elif -10 <= theta < 10:
        direction = 1

    elif 10 <= theta < 30:
        direction = 2

    elif 30 <= theta < 90:
        direction = 3

    elif theta >= 90:
        direction = 4

    else:
        direction = -1  # Not acquired

    return direction
