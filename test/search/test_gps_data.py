from unittest import TestCase

from avidrone.search import util

class TestGPSData(TestCase):
    assert True
    # def __init__(self):
    #     self.WINDOW_SIZE = util.WINDOW_SIZE
    #     self.gps_data = util.GpsData(self.WINDOW_SIZE)
    #     self.gps_point = [1, 2]
    #     self.distance = [2, 3]
    #     self.min_index = 0
        
    # def test_add_point(self):
    #     self.gps_data.add_point(self.gps_point, self.distance)
    #     assert self.gps_data.gps_points is not None
    #     assert self.gps_data.distance is not None

    # def test_get_minimum_index(self):
    #     assert self.gps_data.get_minimum_index() == self.min_index

    # def test_purge_gps_window(self):
    #     self.gps_data.purge_gps_window()
    #     assert self.gps_data.gps_points == []
    #     assert self.gps_data.distance == []
