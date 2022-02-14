#!/usr/bin/env python3
# -*- coding: utf-8 -*-


def flight_mode():
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
