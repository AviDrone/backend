from unittest import TestCase

from UAV import parameters

class Parameters(TestCase):
    @staticmethod
    def test_parameter():
        parameter = parameters.parameter()
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

    @staticmethod
    def test_flight_mode():
        flight_mode = parameters.flight_mode()
        READY = 1
        TAKEOFF = 2
        HOLD = 3
        MISSION = 4
        RETURN_TO_LAUNCH = 5
        LAND = 6
        OFFBOARD = 7
        FOLLOW_ME = 8
        MANUAL = 9
        ALTCTL = 10
        POSCTL = 11
        ACRO = 12
        STABILIZED = 13

        assert flight_mode[0] == READY
        assert flight_mode[1] == TAKEOFF
        assert flight_mode[2] == HOLD
        assert flight_mode[3] == MISSION
        assert flight_mode[4] == RETURN_TO_LAUNCH
        assert flight_mode[5] == LAND
        assert flight_mode[6] == OFFBOARD
        assert flight_mode[7] == FOLLOW_ME
        assert flight_mode[8] == MANUAL
        assert flight_mode[9] == ALTCTL
        assert flight_mode[10] == POSCTL
        assert flight_mode[11] == ACRO
        assert flight_mode[12] == STABILIZED
