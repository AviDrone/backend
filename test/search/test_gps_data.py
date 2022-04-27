from unittest import TestCase

from avidrone.search.gps_data import GPSData


class TestGPSData(TestCase):
    def test_add_point(self):
        WINDOW_SIZE = 5
        gps_data = GPSData(WINDOW_SIZE)
        gps_point = [1, 2]
        distance = [2, 3]

        gps_data.add_point(gps_point, distance)
        assert gps_data.gps_points is not None
        assert gps_data.distance is not None

    def test_get_minimum_index(self):
        WINDOW_SIZE = 5
        gps_data = GPSData(WINDOW_SIZE)
        min_index = 0
        assert gps_data.get_minimum_index() == min_index

    def test_purge_gps_window(self):
        WINDOW_SIZE = 5
        gps_data = GPSData(WINDOW_SIZE)
        gps_point = [1, 2]
        distance = [2, 3]

        gps_data.add_point(gps_point, distance)
        gps_data = GPSData(WINDOW_SIZE)
        gps_data.purge_gps_window()
        assert gps_data.gps_points == []
        assert gps_data.distance == []
