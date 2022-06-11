#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    PRIMARY SEARCH
"""
from __future__ import print_function

import logging
import os
import time

import numpy as np
from dronekit import LocationGlobalRelative, VehicleMode
from pymavlink import mavutil

from search import SEARCH, PRIMARY, SECONDARY
from transceiver.transceiver import TRANSCEIVER
from uav import AVIDRONE

from util import MISSION


# logging
log = logging.getLogger(__name__)
log.setLevel(logging.INFO)
msg = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
file_handler = logging.FileHandler(os.path.join("log", "primary_trial.log"))
file_handler.setFormatter(msg)
log.addHandler(file_handler)

# Primary search parameters
width = SEARCH.width
height = SEARCH.height
length = SEARCH.length
strip_width = TRANSCEIVER.curr_search_strip_width
AVIDRONE.enable_battery_telemetry = True

assert SEARCH.ENABLE_PRIMARY_SEARCH  # STOP if primary search is not enabled

# Start in Guided mode
print("Set GUIDED mode")
AVIDRONE.mode = VehicleMode("GUIDED")

# Set mission for drone angle
print(f"Create a new mission (for current location)")
my_angle = 360 - np.degrees(AVIDRONE.yaw)
print(f"Drone angle: {my_angle}")
