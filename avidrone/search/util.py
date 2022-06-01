#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import asyncio
import datetime
import logging as log
import math
import time

import numpy as np
from dronekit import Command, LocationGlobal, LocationGlobalRelative, VehicleMode

import drone

WITH_TRANSCEIVER = True  # set to false for quicker primary search only operation
if WITH_TRANSCEIVER:
    from transceiver import Transceiver

IS_VERBOSE = False  # for verbose command-line interface output
IS_TEST = False  # for running simulations

# DEFAULT PARAMETERS
MAGNITUDE = 1  # Distance the vehicle goes
ALTITUDE = 15  # Altitude of the flight path
DEGREES = 10  # Amount to rotate in yaw
DEGREE_ERROR = 2  # Number of degrees error for rotation
DISTANCE_ERROR = 0.35  # Error in distance before target reached
LAND_THRESHOLD = 2  # Error in distance before target reached
WINDOW_SIZE = 5  # Size of the gps window original: 5
FLIGHT_MODE = "GUIDED"  # for autonomous missions


class GpsData:
    def __init__(self, window_size):
        self.window_size = window_size
        self.gps_points = []
        self.distance = []

    def add_point(self, new_gps_point, new_distance):
        self.gps_points.insert(0, new_gps_point)
        del self.gps_points[self.window_size :]
        self.distance.insert(0, new_distance)
        del self.distance[self.window_size :]

    def get_minimum_index(self):
        minimum_dist_index = 0

        for i in range(0, len(self.distance)):
            if self.distance[i] < self.distance[minimum_dist_index]:
                minimum_dist_index = i
        return minimum_dist_index

    def purge_gps_window(self):
        self.gps_points.clear()
        self.distance.clear()


class Search:
    def __init__(self):
        aviDrone = drone.vehicle
        self.global_frame = aviDrone.location.global_frame
        # TODO what is this ?
        # self.global_location = LocationGlobal(new_lat, new_lon, original_location.alt)
        # self.distance = (
        #     2
        #     * math.asin(
        #         math.sqrt(
        #             (math.sin((lat_a - lat_b) / 2)) ** 2
        #             + math.cos(lat_a)
        #             * math.cos(lat_b)
        #             * (math.sin((lon_a - lon_b) / 2)) ** 2
        #         )
        #     )
        #     * 1.113195e5
        # )

    @staticmethod
    def get_distance(location_a, location_b):
        lat_a, lat_b = location_a.lat, location_b.lat
        lon_a, lon_b = location_a.lat, location_b.lat

        d_lat = lat_b - lat_a
        d_lon = lon_b - lon_a

        return math.sqrt(math.pow(d_lat, 2) + math.pow(d_lon, 2)) * 1.113195e5

    @staticmethod
    def get_location(original_location, d_north, d_east, d_alt=0):

        earth_radius = 6378137.0  # Radius of "spherical" earth
        # Coordinate offsets in radians
        d_lat = d_north / earth_radius
        d_lon = d_east / (
            earth_radius * math.cos(math.pi * original_location.lat / 180)
        )

        # New position in decimal degrees
        new_lat = original_location.lat + (d_lat * 180 / math.pi)
        new_lon = original_location.lon + (d_lon * 180 / math.pi)

        if original_location.alt:
            return LocationGlobal(new_lat, new_lon, original_location.alt)
        else:
            return LocationGlobal(new_lat, new_lon, d_alt)

    def get_global_pos(self):
        return self.global_frame

    @staticmethod
    def mock_transceiver(uav_pos, beacon_pos):
        using = Transceiver()
        mock_beacon = using.mock_transceiver(uav_pos, beacon_pos)
        return mock_beacon


class Mission:
    def __init__(self):
        from pymavlink import mavutil

        self.aviDrone = drone.vehicle
        self.mavutil = mavutil
        self.global_frame = self.aviDrone.location.global_frame
        self.original_yaw = self.aviDrone.attitude.yaw
        self.heading = -1  # TODO get correct value
        self.relative = False
        self.cw = -1  # TODO get correct value

    @staticmethod
    def better_get_distance_meters(a_location1, a_location2):
        lat1, lat2 = a_location1.lat, a_location2.lat
        lon1, lon2 = a_location1.lon, a_location2.lon

        # 6371.009e3 is the mean radius of the earth that gives us the distance
        # (NOT USED)
        #
        # 1.113195e5 gives us meters
        distance = (
            2
            * math.asin(
                math.sqrt(
                    (math.sin((lat1 - lat2) / 2)) ** 2
                    + math.cos(lat1)
                    * math.cos(lat2)
                    * (math.sin((lon1 - lon2) / 2)) ** 2
                )
            )
            * 1.113195e5
        )
        return distance

    @staticmethod
    def better_get_location_meters(original_location, distance, angle):
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
    def better_goto(distance, angle, vehicle):
        """
        Moves the vehicle to a position d_north metres North and d_east metres East of the current position.
        The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for
        the target position. This allows it to be called with different position-setting commands.
        By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().
        The method reports the distance to target every two seconds.

        Retrieved from this link:
        https://dronekit-python.readthedocs.io/en/latest/examples/guided-set-speed-yaw-demo.html
        """

        vehicle = drone.vehicle
        currentLocation = vehicle.location.global_frame  # was global_relative_frame
        targetLocation = Mission.better_get_location_meters(
            currentLocation, distance, angle
        )
        # targetDistance = better_get_distance_meters(currentLocation, targetLocation)
        # targetDistance = get_distance_meters(currentLocation, targetLocation)
        vehicle.simple_goto(targetLocation)

        # print "DEBUG: targetLocation: %s" % targetLocation
        # print "DEBUG: targetLocation: %s" % targetDistance

        loop_count = 0
        while (
            vehicle.mode.name == "GUIDED"
        ):  # Stop action if we are no longer in guided mode.
            # print "DEBUG: mode: %s" % vehicle.mode.name
            remainingDistance = Mission.better_get_distance_meters(
                vehicle.location.global_frame, targetLocation
            )
            # global_frame was global_relative_frame
            # remainingDistance=get_distance_meters(vehicle.location.global_frame, targetLocation)
            # #global_frame was global_relative_frame
            print("Distance to target: ", remainingDistance)
            # if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.

            loop_count += 1

            if remainingDistance <= 0.35:
                print("Reached target")
                break
            elif loop_count >= 10:
                print("Stuck, skipping target")
                break
            time.sleep(2)

    # Any condition we want to break the primary search can be done in this command.
    # This will be called repeatedly and return true when the break condition is true.
    def break_condition(self):
        next_waypoint = self.aviDrone.commands.next
        if next_waypoint == 40:
            self.aviDrone.mode = VehicleMode("GUIDED")
            print("breaking...")
            return True
        return False

    def condition_yaw(self):
        heading = self.heading
        original_yaw = self.aviDrone.attitude.yaw
        if self.relative:
            is_relative = 1  # yaw relative to direction of travel
        else:
            is_relative = 0  # yaw is an absolute angle

        if self.heading < 0:
            heading = abs(heading)
            self.cw = -1
        else:
            self.cw = 1

        msg = self.aviDrone.message_factory.command_long_encode(
            0,  # target system
            0,  # target component
            self.mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            heading,  # param 1, yaw in degrees
            0,  # param 2, yaw speed deg/s
            self.cw,  # param 3, direction -1 ccw, 1 cw
            is_relative,  # param 4, relative offset 1, absolute angle 0
            0,
            0,
            0,
        )  # param 5 ~ 7 not used
        self.aviDrone.send_mavlink(msg)  # send command to vehicle

    @staticmethod
    def forward_calculation():
        flight_direction = []
        yaw = drone.vehicle.attitude.yaw

        print(yaw)
        flight_direction.append(MAGNITUDE * math.cos(yaw))
        flight_direction.append(MAGNITUDE * math.sin(yaw))

        return flight_direction

    def go_to_location(self, distance, angle):
        current_location = self.aviDrone.location.global_frame
        target_location = Search.get_location(current_location, distance, angle)
        self.aviDrone.simple_goto(target_location)

        loop_count = 0
        while self.aviDrone.mode.name == "GUIDED":
            remaining_distance = Search.get_distance(
                self.aviDrone.location.global_frame, target_location
            )
            print("Distance to target: ", remaining_distance)
            loop_count += 1
            if remaining_distance <= 0.35:
                print("Reached target")
                break
            elif loop_count >= 10:
                print("Stuck, skipping target")
                break
            time.sleep(2)

    def simple_goto_wait(self, go_to_checkpoint):
        self.aviDrone.simple_goto(go_to_checkpoint)
        global_frame = self.aviDrone.location.global_frame
        distance = get_distance_metres(
            Search.get_global_pos(global_frame), go_to_checkpoint
        )

        while distance >= DISTANCE_ERROR and self.aviDrone.mode.name == "GUIDED":
            print(distance)
            distance = get_distance_metres(
                Search.get_global_pos(global_frame), go_to_checkpoint
            )
            time.sleep(1)

        if self.aviDrone.mode.name != "GUIDED":
            self.aviDrone.simple_goto(self.aviDrone.location.global_frame)
            print("Halting simple_go_to")

        print("Checkpoint reached")

    def start(self):
        log.info("-- Waiting for vehicle to start...")
        while not self.aviDrone.is_armable:
            time.sleep(1)

        self.aviDrone.mode = VehicleMode("GUIDED")
        self.aviDrone.armed = True

        log.info("-- Arming...")
        while not self.aviDrone.is_armable:
            time.sleep(1)

        if self.aviDrone.armed:
            log.info(f"-- Is armed: {self.aviDrone.armed}")
            self.takeoff_to_altitude()

        log.info("-- Waiting for GUIDED mode...")
        while self.aviDrone.mode.name != "GUIDED":
            time.sleep(1)

    def takeoff_to_altitude(self):
        self.aviDrone.simple_takeoff(ALTITUDE)
        log.info(f"-- Taking off to altitude (m): {ALTITUDE} \n")

        while True:
            current_alt = self.aviDrone.location.global_relative_frame.alt
            if current_alt >= ALTITUDE * 0.95:
                log.info(f"-- Reached {ALTITUDE}m")
                break
            time.sleep(1)


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


def get_distance_metres(a_location1, a_location2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    d_lat = a_location2.lat - a_location1.lat
    d_long = a_location2.lon - a_location1.lon
    return math.sqrt((d_lat * d_lat) + (d_long * d_long)) * 1.113195e5


def get_location_metres(original_location, dNorth, dEast):
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
    return LocationGlobal(new_lat, new_lon, original_location.alt)


def get_location_metres_with_alt(original_location, dNorth, dEast, newAlt):
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


def get_range(totalLength, dLength):
    return (totalLength / dLength) * 2
