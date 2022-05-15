from cgi import test
import math
import random
from re import S
from unittest import TestCase

import numpy as np

from avidrone.transceiver import util


class TestTransceiver(TestCase):
    uav = [3, 20, 5]
    beacon = [25, 22, 0]

    displacement = util.get_displacement(uav[0], beacon[0], uav[1], beacon[1], uav[2], beacon[2])
    print(displacement)

    def test_get_displacement(self):
        assert TestTransceiver.displacement[0] == 22  # dx
        assert TestTransceiver.displacement[1] == 2  # dy
        assert TestTransceiver.displacement[2] == -5  # dz

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