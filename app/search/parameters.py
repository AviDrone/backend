#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    DEFAULT PARAMETERS
"""

IS_TEST = True  # for running simulations
MISSION_TIMEOUT = False  # for returning home

ALTITUDE = 15  # Altitude of the flight path
DEGREES = 10  # Amount to rotate in yaw
DEGREE_ERROR = 2  # Number of degrees error for rotation
DISTANCE_ERROR = 0.35  # Error in distance before target reached
LAND_THRESHOLD = 2  # Error in distance before landing
MAGNITUDE = 1  # Distance the vehicle goes forward per command
WINDOW_SIZE = 5  # Size of the gps window
