#!/usr/bin/env python3
# -*- coding: utf-8 -*-


def parameter():
    """"
    Initial AviDrone Parameters for search mission.
    """""

    MAGNITUDE = 1  # Param 0, Set the distance the drone goes
    HEIGHT = 4  # Param 1, Set the height of the flight path
    DEGREES = 10  # Param 2,Set the amount to rotate in yaw
    DEGREE_ERROR = 2  # Param 3, Number of degrees error for rotation
    DISTANCE_ERROR = 0.35  # Param 4, Error in distance before target reached
    LAND_THRESHOLD = 2  # Param 5, Error in distance before target reached
    WINDOW_SIZE = 5  # Param 6, Set the size of the gps window original: 5

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
