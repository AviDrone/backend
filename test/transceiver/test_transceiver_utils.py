from unittest import TestCase

import numpy as np
from avidrone.transceiver import utils


class Search(TestCase):
    def test_get_displacement(self):
        x_1 = 10
        x_2 = 5

        y_1 = 13
        y_2 = 12

        z_1 = 0
        z_2 = 0
        displacement = utils.get_displacement(x_1, x_2, y_1, y_2, z_1, z_2)
        assert displacement == [-5, -1, 0]

    def test_get_euclidean_distance(self):
        x_1 = 10
        x_2 = 5

        y_1 = 13
        y_2 = 12

        z_1 = 0
        z_2 = 0
        displacement = utils.get_displacement(x_1, x_2, y_1, y_2, z_1, z_2)
        euclidean_distance = utils.get_euclidean_distance(displacement)
        assert euclidean_distance == 0.0014792899408284023
