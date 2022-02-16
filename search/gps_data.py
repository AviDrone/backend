#!/usr/bin/python


class GPSData:
    def __init__(self, window_size):
        self.window_size = window_size
        self.gps_points = []
        self.distance = []

    def add_point(self, new_gps_point, new_distance):
        self.gps_points.insert(
            0, new_gps_point
        )  # inserts new_gps_point at the beginning
        # deletes everything after window_size
        del self.gps_points[self.window_size :]

        # inserts new_distance at the beginning
        self.distance.insert(0, new_distance)
        # deletes everything after window_size
        del self.distance[self.window_size :]

    def get_minimum_index(self):
        minimum_distance_index = 0  # stores index of minimum distance

        for i in range(0, len(self.distance)):
            if self.distance[i] < self.distance[minimum_distance_index]:
                minimum_distance_index = i
        return minimum_distance_index

    def purge_gps_window(self):
        self.gps_points.clear()
        self.distance.clear()
