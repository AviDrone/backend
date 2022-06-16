#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest

from app.search import parameters as p


class TestParameters(unittest.TestCase):
    def __init__(self):
        super().__init__()
        self.T_ALTITUDE = 15
        self.T_DEGREES = 10
        self.T_DEGREE_ERROR = 2
        self.T_DISTANCE_ERROR = 0.35
        self.T_LAND_THRESHOLD = 2
        self.T_MAGNITUDE = 1
        self.T_WINDOW_SIZE = 5

    def test_parameters(self):
        altitude_msg = "Incorrect altitude!"
        degrees_msg = "Incorrect degrees!"
        degree_error_msg = "Incorrect degree error!"
        distance_error_msg = "Incorrect distance error!"
        land_threshold_msg = "Incorrect degrees!"
        magnitude_msg = "Incorrect magnitude!"
        window_size_msg = "Incorrect window size!"

        self.assertFalse(p.MISSION_TIMEOUT)
        # self.assertEquals(self.T_ALTITUDE, p.ALTITUDE, altitude_msg)
        # self.assertEquals(self.T_DEGREES, p.DEGREES, degrees_msg)
        # self.assertEquals(self.T_DEGREE_ERROR, p.DEGREE_ERROR, degree_error_msg)
        # self.assertEquals(
        #     self.T_DISTANCE_ERROR, p.DISTANCE_ERROR, distance_error_msg
        # )
        # self.assertEquals(
        #     self.T_LAND_THRESHOLD, p.AND_THRESHOLD, land_threshold_msg
        # )
        # self.assertEquals(self.T_MAGNITUDE, p.MAGNITUDE, magnitude_msg)
        # self.assertEquals(self.T_WINDOW_SIZE, p.WINDOW_SIZE, window_size_msg)


if __name__ == "__main__":
    unittest.main()
