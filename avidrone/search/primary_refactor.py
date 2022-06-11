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
from search import PRIMARY, SEARCH, SECONDARY
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
SEARCH.ENABLE_PRIMARY_SEARCH = True

if SEARCH.ENABLE_PRIMARY_SEARCH:
    PRIMARY.is_enabled = True
# Start in Guided mode
print("Set GUIDED mode")
AVIDRONE.mode = VehicleMode("GUIDED")

# Set mission for drone angle
print(f"Create a new mission (for current location)")
print(f"Drone angle: {AVIDRONE.angle}")


assert PRIMARY.is_enabled # STOP if primary search is not enabled
PRIMARY.rectangular(width, TRANSCEIVER.curr_search_strip_width, length, AVIDRONE.angle, height)
