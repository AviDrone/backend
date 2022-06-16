#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest

from app.search import parameters


class TestParameters(unittest.TestCase):
    def __init__(self):
        self.MISSION_TIMEOUT = False
        self.ALTITUDE = 15
        self.DEGREES = 10
        self.DEGREE_ERROR = 2
        self.DISTANCE_ERROR = 0.35
        self.LAND_THRESHOLD = 2
        self.MAGNITUDE = 1
        self.WINDOW_SIZE = 5

    def test_parameters(self):
        altitude_msg = "Incorrect altitude!"
        degrees_msg = "Incorrect degrees!"
        degree_error_msg = "Incorrect degree error!"
        distance_error_msg = "Incorrect distance error!"
        land_threshold_msg = "Incorrect degrees!"
        magnitude_msg = "Incorrect magnitude!"
        window_size_msg = "Incorrect window size!"

        self.assertFalse(parameters.MISSION_TIMEOUT)
        self.assertEquals(self.ALTITUDE, parameters.ALTITUDE, altitude_msg)
        self.assertEquals(self.DEGREES, parameters.DEGREES, degrees_msg)
        self.assertEquals(self.DEGREE_ERROR, parameters.DEGREE_ERROR, degree_error_msg)
        self.assertEquals(
            self.DISTANCE_ERROR, parameters.DISTANCE_ERROR, distance_error_msg
        )
        self.assertEquals(
            self.LAND_THRESHOLD, parameters.AND_THRESHOLD, land_threshold_msg
        )
        self.assertEquals(self.MAGNITUDE, parameters.MAGNITUDE, magnitude_msg)
        self.assertEquals(self.WINDOW_SIZE, parameters.WINDOW_SIZE, window_size_msg)


if __name__ == "__main__":
    unittest.main()
