#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    UTILITIES
"""
from __future__ import print_function

import logging
import math
import os
import time

import numpy as np
import utm
from dronekit import LocationGlobal, VehicleMode
from parameters import DISTANCE_ERROR, MAGNITUDE, WINDOW_SIZE
from pymavlink import mavutil
from uav import AVIDRONE

# logging
log = logging.getLogger(__name__)
log.setLevel(logging.INFO)
msg = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
file_handler = logging.FileHandler(os.path.join("log", "util.log"))
file_handler.setFormatter(msg)
log.addHandler(file_handler)


class GpsData:
    """
    GPS Data

    """

    def __init__(self):
        self.name = "GPS Data"
        self.window_size = WINDOW_SIZE
        self.gps_points = []
        self.distance = []

    def add_point(self, new_gps_point, new_distance):
        self.gps_points.insert(0, new_gps_point)
        del self.gps_points[self.window_size :]
        self.distance.insert(0, new_distance)
        del self.distance[self.window_size :]

    def get_minimum_index(self):
        min_index = 0
        for i in range(0, len(self.distance)):
            if self.distance[i] < self.distance[min_index]:
                min_index = i
        return min_index

    def purge_gps_window(self):
        self.gps_points.clear()
        self.distance.clear()


GPS_DATA = GpsData()  # singleton


class Mission:
    def __init__(self):
        self.name = "Mission"
        self.mavutil = mavutil
        self.global_frame = AVIDRONE.altitude
        self.original_yaw = AVIDRONE.yaw
        self.angle = 360 - AVIDRONE.yaw
        self._filename = None

    @staticmethod
    def download_mission():
        """
        Downloads the current mission and returns it in a list.

        """

        mission = []
        commands = AVIDRONE.commands
        commands.download()
        commands.wait_ready()
        for command in commands:
            mission.append(command)
        return mission

    def save_mission(self, text_file):
        """
        Save a mission in the Waypoint file format:
        (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).

        """

        missions = self.download_mission()
        output = "QGC WPL 110\n"
        for cmd in missions:
            command_line = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
                cmd.seq,
                cmd.current,
                cmd.frame,
                cmd.command,
                cmd.param1,
                cmd.param2,
                cmd.param3,
                cmd.param4,
                cmd.x,
                cmd.y,
                cmd.z,
                cmd.autocontinue,
            )
            output += command_line
        with open(text_file, "w") as file_:
            file_.write(output)

    def condition_yaw(self, heading, relative):
        """
        Turns the UAV left or right based on a condition.

        """

        if relative:
            is_relative = True  # yaw relative to direction of travel
        else:
            is_relative = False  # yaw is an absolute angle

        if heading < 0:
            heading = abs(heading)
            cw = -1
        else:
            cw = 1

        msg = AVIDRONE.message_factory.command_long_encode(
            0,  # target system
            0,  # target component
            self.mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            heading,  # param 1, yaw in degrees
            0,  # param 2, yaw speed deg/s
            cw,  # param 3, direction -1 ccw, 1 cw
            is_relative,  # param 4, relative offset 1, absolute angle 0
            0,
            0,
            0,
        )  # param 5 ~ 7 not used
        AVIDRONE.send_mavlink(msg)  # send command to vehicle


MISSION = Mission()  # Singleton


class Vector:
    def __init__(self):
        vector_1 = []
        vector_2 = []
        self.vector_1 = np.asarray(vector_1)
        self.vector_2 = np.asarray(vector_2)

    @staticmethod
    def rotate_cloud(Points, V1, V2):
        # V1 is the current vector which the coordinate system is aligned to
        # V2 is the vector we want the system aligned to
        # Points is an (n,3) array of n points (x,y,z)
        V1 = np.asarray(V1)
        V2 = np.asarray(V2)

        # Normalize V1 and V2 in case they aren't already
        V1Len = (V1[0] ** 2 + V1[1] ** 2 + V1[2] ** 2) ** 0.5
        V2Len = (V2[0] ** 2 + V2[1] ** 2 + V2[2] ** 2) ** 0.5
        V1 = V1 / V1Len
        V2 = V2 / V2Len

        # Calculate the vector cross product
        V1V2Cross = np.cross(V1, V2)
        V1V2CrossNorm = (
            V1V2Cross[0] ** 2 + V1V2Cross[1] ** 2 + V1V2Cross[2] ** 2
        ) ** 0.5
        V1V2CrossNormalized = V1V2Cross / V1V2CrossNorm

        # Dot product
        V1V2Dot = np.dot(V1, V2)
        V1Norm = (V1[0] ** 2 + V1[1] ** 2 + V1[2] ** 2) ** 0.5
        V2Norm = (V2[0] ** 2 + V2[1] ** 2 + V2[2] ** 2) ** 0.5

        # angle between the vectors
        Theta = np.arccos(V1V2Dot / (V1Norm * V2Norm))

        # Using Rodriguez's rotation formula (wikipedia):
        e = V1V2CrossNormalized
        pts_rotated = np.empty((len(Points), 3))
        if np.size(Points) == 3:
            p = Points
            p_rotated = (
                np.cos(Theta) * p
                + np.sin(Theta) * (np.cross(e, p))
                + (1 - np.cos(Theta)) * np.dot(e, p) * e
            )
            pts_rotated = p_rotated
        else:
            for i, p in enumerate(Points):
                p_rotated = (
                    np.cos(Theta) * p
                    + np.sin(Theta) * (np.cross(e, p))
                    + (1 - np.cos(Theta)) * np.dot(e, p) * e
                )
                pts_rotated[i] = p_rotated
        return pts_rotated

    @staticmethod
    def rotate_vector(vector, angle):
        # vector is the vector being rotated
        # angle is used to rotate vector and is given in degrees

        # Convert angle to radians
        Angle_Rad = np.radians(angle)

        # rotation matrix
        # See https://en.wikipedia.org/wiki/Rotation_matrix for more information
        r = np.array(
            (
                (np.cos(Angle_Rad), -np.sin(Angle_Rad)),
                (np.sin(Angle_Rad), np.cos(Angle_Rad)),
            )
        )

        # we only care about x and y, not z
        a_vector = (vector[0], vector[1])
        a_vector = np.asarray(a_vector)

        # vector after rotation
        rotated = r.dot(a_vector)

        # return 3D vector
        NewVector = (rotated[0], rotated[1], vector[2])

        return NewVector

    @staticmethod
    def get_range(total_length, d_length):
        return (total_length / d_length) * 2


VECTOR = Vector()


class Navigation:
    def __init__(self):
        self.mavutil = mavutil
        self.global_frame = AVIDRONE.altitude
        self.original_yaw = AVIDRONE.yaw
        self.angle = 360 - AVIDRONE.yaw
        self.relative = False

    @staticmethod
    def arm_and_takeoff(target_altitude):
        # Pre-arm check: Prevent user try to arm until autopilot is ready
        log.info(" Waiting for vehicle to initialize...")
        while not AVIDRONE.is_armable:
            log.debug("Arming motors")  # Copter should arm in GUIDED mode
            time.sleep(1)
            AVIDRONE.mode = VehicleMode("GUIDED")
            AVIDRONE.armed = True

            while not AVIDRONE.armed:
                log.info(" Waiting for arming...")
                time.sleep(1)

        log.warning("Taking off!")
        AVIDRONE.simple_takeoff(target_altitude)  # Take off to target altitude

        # Wait until the vehicle reaches a safe height before continuing
        # otherwise the command after Vehicle.simple_takeoff will execute immediately.
        while True:
            print(" Altitude: ", AVIDRONE.altitude)
            time.sleep(1)
            if (
                AVIDRONE.altitude >= target_altitude * 0.95
            ):  # Trigger just below target alt.
                print("Reached target altitude")
                break

    def go_to(self, distance, angle):
        """
        Reference:
        https://dronekit-python.readthedocs.io/en/latest/examples/guided-set-speed-yaw-demo.html
        """

        curr_location = AVIDRONE.location.global_frame
        targetLocation = self.get_location_meters(curr_location, distance, angle)
        AVIDRONE.simple_goto(targetLocation)

        counter = 0
        while AVIDRONE.mode == "GUIDED":
            remaining_distance = self.get_distance_meters(
                AVIDRONE.location.global_frame, targetLocation
            )
            print("Distance to target: ", remaining_distance)
            counter += 1

            if remaining_distance <= 0.35:
                print("Reached target")
                break
            elif counter >= 10:
                print("Stuck, skipping target")
                break
            time.sleep(2)

    @staticmethod
    def get_location_meters(original_location, distance, angle):
        lat = math.asin(
            math.sin(original_location.lat) * math.cos(distance)
            + math.cos(original_location.lat) * math.sin(distance) * math.cos(angle)
        )

        d_lon = math.atan2(
            math.sin(angle) * math.sin(distance) * math.cos(original_location.lat),
            math.cos(distance) - math.sin(original_location.lat) * math.sin(lat),
        )

        lon = (original_location.lon - d_lon + math.pi) % (2 * math.pi) - math.pi

        return LocationGlobal(lat, lon, original_location.alt)

    @staticmethod
    def get_location_meters_with_alt(original_location, dNorth, dEast, newAlt):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
        specified `original_location`. The returned Location has the same `alt` value
        as `original_location`.
        The function is useful when you want to move the vehicle around specifying locations relative to
        the current vehicle position.
        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius = 6378137.0  # Radius of "spherical" earth
        # Coordinate offsets in radians
        dLat = dNorth / earth_radius
        dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

        # New position in decimal degrees
        new_lat = original_location.lat + (dLat * 180 / math.pi)
        new_lon = original_location.lon + (dLon * 180 / math.pi)

        return LocationGlobal(new_lat, new_lon, newAlt)

    def simple_goto_wait(self, go_to_checkpoint):
        AVIDRONE.simple_goto(go_to_checkpoint)
        distance = self.get_distance_meters(AVIDRONE.location, go_to_checkpoint)

        while distance >= DISTANCE_ERROR and AVIDRONE.mode == "GUIDED":
            distance = self.get_distance_meters(AVIDRONE.location, go_to_checkpoint)
            log.info(distance)
            time.sleep(1)

        if AVIDRONE.mode != "GUIDED":
            AVIDRONE.mode.simple_goto(AVIDRONE.location)
            log.warning("Halting simple_go_to")
        log.info("Checkpoint reached")

    @staticmethod
    def get_distance_meters(a, b):
        lat_a, lat_b = a.lat, b.lat
        lon_a, lon_b = a.lon, b.lon
        distance = (
            2
            * math.asin(
                math.sqrt(
                    (math.sin((lat_a - lat_b) / 2)) ** 2
                    + math.cos(lat_a)
                    * math.cos(lat_b)
                    * (math.sin((lon_a - lon_b) / 2)) ** 2
                )
            )
            * 1.113195e5
        )
        return distance  # in meters

    @staticmethod
    def utm2latlon(utm):
        _latlon = utm.to_latlon(utm)
        return _latlon

    @staticmethod
    def latlon2utm(latlon):
        _utm = utm.from_latlon(latlon)
        return _utm

    @staticmethod
    def add_rel_pos(utm_pos, rel_pos):
        new_ = [
            utm_pos[0] + rel_pos[0],
            utm_pos[1] + rel_pos[1],
            utm_pos[2],
            utm_pos[3],
        ]
        return new_

    @staticmethod
    def forward_calculation():
        flight_direction = [
            MAGNITUDE * math.cos(AVIDRONE.yaw),
            MAGNITUDE * math.sin(AVIDRONE.yaw),
        ]
        return flight_direction


NAVIGATION = Navigation()
