from unittest import TestCase

from UAV import parameters


class Parameters(TestCase):
    @staticmethod
    def test_parameter():
        parameter = parameters.parameter()
        MAGNITUDE = 1  # Parameter 0
        HEIGHT = 4  # Parameter 1
        DEGREES = 10  # Parameter 2
        DEGREE_ERROR = 2  # Parameter 3
        DISTANCE_ERROR = 0.35  # Parameter 4
        LAND_THRESHOLD = 2  # Parameter 5
        WINDOW_SIZE = 5  # Parameter 6

        assert parameter[0] == MAGNITUDE
        assert parameter[1] == HEIGHT
        assert parameter[2] == DEGREES
        assert parameter[3] == DEGREE_ERROR
        assert parameter[4] == DISTANCE_ERROR
        assert parameter[5] == LAND_THRESHOLD
        assert parameter[6] == WINDOW_SIZE
