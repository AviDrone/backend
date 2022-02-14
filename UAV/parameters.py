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


def flight_mode():
    """
    For more info on flight modes:
    https://docs.px4.io/master/en/config/flight_mode.html
    http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/plugins/telemetry.html
    """

    READY = 1
    TAKEOFF = 2
    HOLD = 3
    MISSION = 4
    RETURN_TO_LAUNCH = 5
    LAND = 6
    OFFBOARD = 7
    FOLLOW_ME = 8
    MANUAL = 9
    ALTCTL = 10
    POSCTL = 11
    ACRO = 12
    STABILIZED = 13

    flight_mode = [
        READY,
        TAKEOFF,
        HOLD,
        MISSION,
        RETURN_TO_LAUNCH,
        LAND,
        OFFBOARD,
        FOLLOW_ME,
        MANUAL,
        ALTCTL,
        POSCTL,
        ACRO,
        STABILIZED,
    ]

    return flight_mode
