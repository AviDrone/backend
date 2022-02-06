from unittest import TestCase
from search import functions

class Test(TestCase):
    def test_param(self):
        parameter = functions.param()
        MAGNITUDE = 1  # Param 0
        HEIGHT = 4  # Param 1
        DEGREES = 10  # Param 2
        DEGREE_ERROR = 2  # Param 3
        DISTANCE_ERROR = 0.35  # Param 4
        LAND_THRESHOLD = 2  # Param 5
        WINDOW_SIZE = 5  # Param 6

        assert(parameter[0] == MAGNITUDE)
        assert (parameter[1] == HEIGHT)
        assert (parameter[2] == DEGREES)
        assert (parameter[3] == DEGREE_ERROR)
        assert (parameter[4] == DISTANCE_ERROR)
        assert (parameter[5] == LAND_THRESHOLD)
        assert (parameter[6] == WINDOW_SIZE)
