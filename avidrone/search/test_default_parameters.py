from unittest import TestCase

from avidrone.search import default_parameters

class Parameter(TestCase):
    def test_parameter(self):
        parameter = default_parameters.parameter()
        MAGNITUDE = 1
        HEIGHT = 4
        DEGREES = 10
        DEGREE_ERROR = 2
        DISTANCE_ERROR = 0.35
        LAND_THRESHOLD = 2
        WINDOW_SIZE = 5

        assert parameter[0] == MAGNITUDE
        assert parameter[1] == HEIGHT
        assert parameter[2] == DEGREES
        assert parameter[3] == DEGREE_ERROR
        assert parameter[4] == DISTANCE_ERROR
        assert parameter[5] == LAND_THRESHOLD
        assert parameter[6] == WINDOW_SIZE
