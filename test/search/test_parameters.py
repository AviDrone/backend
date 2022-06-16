#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pytest
from unittest import TestCase
from app.search import parameters as p

class TestParameters(TestCase):

    def test_parameters(self):
        T_ALTITUDE = 15
        T_DEGREES = 10
        T_DEGREE_ERROR = 2
        T_DISTANCE_ERROR = 0.35
        T_LAND_THRESHOLD = 2
        T_MAGNITUDE = 1
        T_WINDOW_SIZE = 5

        altitude_msg = "Incorrect altitude!"
        degrees_msg = "Incorrect degrees!"
        degree_error_msg = "Incorrect degree error!"
        distance_error_msg = "Incorrect distance error!"
        land_threshold_msg = "Incorrect degrees!"
        magnitude_msg = "Incorrect magnitude!"
        window_size_msg = "Incorrect window size!"

        self.assertFalse(p.MISSION_TIMEOUT)
        self.assertEquals(T_ALTITUDE, p.ALTITUDE, altitude_msg)
        self.assertEquals(T_DEGREES, p.DEGREES, degrees_msg)
        self.assertEquals(T_DEGREE_ERROR, p.DEGREE_ERROR, degree_error_msg)
        self.assertEquals(
            T_DISTANCE_ERROR, p.DISTANCE_ERROR, distance_error_msg
        )
        self.assertEquals(
            T_LAND_THRESHOLD, p.LAND_THRESHOLD, land_threshold_msg
        )
        self.assertEquals(T_MAGNITUDE, p.MAGNITUDE, magnitude_msg)
        self.assertEquals(T_WINDOW_SIZE, p.WINDOW_SIZE, window_size_msg)

