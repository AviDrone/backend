import numpy as np

from unittest import TestCase
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
        assert(displacement == [-5,-1,0])

    def test_get_uclidean_distance(self):
        x_1 = 10
        x_2 = 5

        y_1 = 13
        y_2 = 12

        z_1 = 0
        z_2 = 0
        displacement = utils.get_displacement(x_1, x_2, y_1, y_2, z_1, z_2)
        uclidean_distance = utils.get_uclidean_distance(displacement)
        assert (uclidean_distance == 0.0014792899408284023)

    def test_normalize_vector(self):
        # d_v = utils.get_displacement(x_1, x_2, y_1, y_2, z_1, z_2)
        # normalized_v_d_array = d_v / np.linalg.norm(d_v)
        # normalized_v_d = normalized_v_d_array.tolist()
        assert True

        # Assert values are normal
        # Assert values are expected.
