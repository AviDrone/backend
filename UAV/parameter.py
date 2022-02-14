#!/usr/bin/env python3
# -*- coding: utf-8 -*-


def parameter():
    """
    Initial AviDrone Parameters for search mission
    """

    MAGNITUDE = 1  # Set the distance the drone goes
    HEIGHT = 4  # Set the height of the flight path
    DEGREES = 10  # Set the amount to rotate in yaw
    DEGREE_ERROR = 2  # Number of degrees error for rotation
    DISTANCE_ERROR = 0.35  # Error in distance before target reached
    LAND_THRESHOLD = 2  # Error in distance before target reached
    WINDOW_SIZE = 5  # Set the size of the gps window original: 5

    params = [
        MAGNITUDE,
        HEIGHT,
        DEGREES,
        DEGREE_ERROR,
        DISTANCE_ERROR,
        LAND_THRESHOLD,
        WINDOW_SIZE,
    ]

    return params
