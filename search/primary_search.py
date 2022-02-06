#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

# For interfacing with the C code
import ctypes
import math
import time

from direction_distance import DirectionDistance
from dronekit import (
    LocationGlobal,
    LocationGlobalRelative,
    VehicleMode,
    connect,
)
from gps_data import GPSData
from lattitude_longitude import better_get_distance_meters, better_goto
from pymavlink import mavutil
