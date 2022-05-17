from cgi import test
import math
import random
from re import S
from unittest import TestCase

import numpy as np

from avidrone.search.transceiver import util


class TestTransceiver(TestCase):
    random.seed(492)
    uav = [12, 120, 25]
    beacon = [60, 60, 1]

    displacement = util.get_displacement(uav[0], beacon[0], uav[1], beacon[1], uav[2], beacon[2])
    theta = util.get_angle(displacement)
    print(theta)

    def test_get_displacement(self):
        assert TestTransceiver.displacement[0] == 48  # dx
        assert TestTransceiver.displacement[1] == -60  # dy
        assert TestTransceiver.displacement[2] == -24  # dz

    def test_get_distance_xy(self):
        test_displacement = (
            TestTransceiver.displacement[0] ** 2 +
            TestTransceiver.displacement[1] ** 2
        ) ** -2
        assert test_displacement == util.get_distance_xy(TestTransceiver.displacement)

    def test_get_distance_xyz(self):
        test_displacement = (
            TestTransceiver.displacement[0] ** 2 +
            TestTransceiver.displacement[1] ** 2 +
            TestTransceiver.displacement[2] ** 2
        ) ** -2
        assert test_displacement == util.get_distance_xyz(TestTransceiver.displacement)

    def test_normalize(self):
        d_v = TestTransceiver.displacement / np.linalg.norm(TestTransceiver.displacement)
        d_v_normal = d_v.tolist()

        test_normal_vector = util.normalize(TestTransceiver.displacement)
        assert test_normal_vector == d_v_normal

    def test_get_angle(self):
        assert True

    def test_get_direction(self):
        assert True
