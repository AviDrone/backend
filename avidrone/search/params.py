#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    SEARCH PARAMETERS
"""

IS_TEST = False  # for running simulations
WITH_TRANSCEIVER = True  # set to false for quicker primary search only operation
IS_VERBOSE = False  # for verbose output

# DEFAULT SETTINGS
MAGNITUDE = 1  # Distance the vehicle goes forward per command
ALTITUDE = 15  # Altitude of the flight path
DEGREES = 10  # Amount to rotate in yaw
DEGREE_ERROR = 2  # Number of degrees error for rotation
DISTANCE_ERROR = 0.35  # Error in distance before target reached
LAND_THRESHOLD = 2  # Error in distance before landing
WINDOW_SIZE = 5  # Size of the gps window
