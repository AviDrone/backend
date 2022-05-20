import math
import random
from cgi import test
from re import S
from unittest import TestCase

import magpylib as magpy
import matplotlib.pyplot as plt
import numpy as np
from avidrone.search.transceiver import transceiver_EM_field


class TestTransceiver(TestCase):
    def test_grid(self):
        # create grid
        test_ts = np.linspace(-100, 100, 100)
        test_grid = np.array([[(x, 0, z) for x in test_ts] for z in test_ts])

        assert True

    def test_coil_1(self):

        assert True

    def test_coil_2(self):

        assert True

    def test_figure_styling(self):

        assert True
