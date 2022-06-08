#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    SEARCH UTIL
"""

from __future__ import print_function

import argparse
import asyncio
import datetime
import logging
import math
import os
import time
from backend.avidrone.search.primary_functions import save_mission
from backend.avidrone.search.uav import AVIDRONE
import drone
import numpy as np
from dronekit import (
    Command,
    LocationGlobal,
    LocationGlobalRelative,
    VehicleMode,
)
from pymavlink import mavutil

# logging
log = logging.getLogger(__name__)
log.setLevel(logging.INFO)
msg = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
file_handler = logging.FileHandler(os.path.join("log", "util.log"))
file_handler.setFormatter(msg)
log.addHandler(file_handler)

WITH_TRANSCEIVER = False  # set to false for quicker primary search only operation
if WITH_TRANSCEIVER:
    from transceiver.transceiver import Transceiver

IS_VERBOSE = False  # for verbose command-line interface output
IS_TEST = False  # for running simulations

ENABLE_PRIMARY_SEARCH = False
ENABLE_SECONDARY_SEARCH = False

# DEFAULT PARAMETERS
MAGNITUDE = 1  # Distance the vehicle goes forward per command
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


# Search superclass
class Search:
    def __init__(self):
        avidrone = drone.vehicle
        self.global_frame = avidrone.location.global_frame

    def get_distance(self, location_a, location_b):
        lat_a, lat_b = location_a.lat, location_b.lat
        lon_a, lon_b = location_a.lat, location_b.lat

        d_lat = lat_b - lat_a
        d_lon = lon_b - lon_a

        return math.sqrt(math.pow(d_lat, 2) + math.pow(d_lon, 2)) * 1.113195e5

    # TODO add utm/lat-long conversion functions here

    def get_location(self, original_location, d_north, d_east, d_alt=0):
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
    
    
    def mock_transceiver(self, uav_pos, beacon_pos):
        mock_beacon = Transceiver.mock_transceiver(uav_pos, beacon_pos)
        return mock_beacon

# Search subclasses

class Primary(Search):
# TODO Remove this comment: implement
    def __init__(self):
        self.commands = AVIDRONE.commands
        self.start_wp = Command(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        10,
    )
        self.max_range = int(get_range(-1, -1))  # TODO get from parent search class

    def rectangular(self,angle, location, width, length, height=0):
        commands = []
        _commands = self.commands
        max_range = self.max_range
        v_dist  = h_dist = step = 0
        arr = []
        
        _commands.clear()  # Clears any existing commands
        _commands.add(self.start_wp)
        for _ in range(1, max_range):
            if step == 0:
                h_dist = 0
            elif (step == 1) or (step == 3):
                v_dist += length
            else:
                h_dist = width
            step += 1
            step = step % 4
            arr.append([h_dist, v_dist, 0])
            
        # Rotation
        initial_vector = (arr[1][0], arr[1][1], 0)
        initial_vector = np.asarray(initial_vector)
        final_vector = Vector.rotate_vector(initial_vector, angle)
        
        if np.array_equal(final_vector, initial_vector):
            # do not rotate if equal
            rotated_vector = np.array(arr)
        else:
            # otherwise, rotate
            rotated_vector = Vector.rotate_cloud(arr, initial_vector, final_vector)


        for points in rotated_vector:
            point = get_location_metres(location, points[1], points[0], points[2])
            wp_command = Command(
                0,
                0,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                0,
                0,
                0,
                0,
                0,
                point.lat,
                point.lon,
                ALTITUDE,
            )
            _commands.add(wp_command)
            commands.append(_commands)

        if height == 0:
            # add dummy waypoint at final point (lets us know when have reached destination)
            final_wp = Command(
                0,
                0,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                0,
                0,
                0,
                0,
                0,
                point.lat,
                point.lon,
                point.alt,
            )
            
        else:
            # add dummy waypoint at final point (lets us know when have reached destination)
            final_wp = Command(
                0,
                0,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                0,
                0,
                0,
                0,
                0,
                point.lat,
                point.lon,
                0,
            )
            
        _commands.add(final_wp)
        commands.append(_commands)
        log.info(" Upload new commands to vehicle")
        _commands.upload()

# TODO Remove this comment: passes total length, dlength
    def primary(self, len, d_len):
        end_reached = False
        stopping_point = 0
        ENABLE_PRIMARY_SEARCH = True
        
        if  AVIDRONE.mode != "AUTO":
            time.sleep(1)

        while ENABLE_PRIMARY_SEARCH:
            # TODO log battery info
            next_wp = AVIDRONE.commands.next
            # TODO log distance to waypoint
        


class Secondary(Search):
# TODO Remove this comment: implement
    def __init__(self):
        pass

class Mission:
    def __init__(self):
        # TODO Remove this comment: replace with UAV singleton
        self.avidrone = drone.vehicle
        self.mavutil = mavutil
        self.global_frame = self.avidrone.location.global_frame
        self.original_yaw = self.avidrone.attitude.yaw
        self.heading = -1  # TODO get correct value
        self.angle = 360 - np.degrees(self.avidrone.attitude.yaw)
        self.relative = False
        self.cw = -1  # TODO get correct value

    def arm_and_takeoff(self, target_altitude):
        log.debug("Basic pre-arm checks")
        # Don not let the user try to arm until autopilot is ready
        while not self.avidrone.is_armable:
            print(" Waiting for vehicle to initialize...")
            time.sleep(1)

            log.debug("Arming motors")  # Copter should arm in GUIDED mode
            self.avidrone.mode = VehicleMode("GUIDED")
            self.avidrone.armed = True

        while not self.avidrone.armed:
            log.info(" Waiting for arming...")
            time.sleep(1)

        log.warning("Taking off!")
        self.avidrone.simple_takeoff(target_altitude)  # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", self.avidrone.location.global_relative_frame.alt)
            time.sleep(1)
            if (
                self.avidrone.location.global_relative_frame.alt
                >= target_altitude * 0.95
            ):  # Trigger just below target alt.
                print("Reached target altitude")
                break

    def better_get_distance_meters(self, a_location1, a_location2):
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

    def better_get_location_meters(self, original_location, distance, angle):
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

    def better_goto(self, distance, angle):
        """
        Moves the vehicle to a position d_north metres North and d_east metres East of the current position.
        The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for
        the target position. This allows it to be called with different position-setting commands.
        By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().
        The method reports the distance to target every two seconds.

        Retrieved from this link:
        https://dronekit-python.readthedocs.io/en/latest/examples/guided-set-speed-yaw-demo.html
        """

        currentLocation = (
            self.avidrone.location.global_frame
        )  # was global_relative_frame
        targetLocation = self.better_get_location_meters(
            currentLocation, distance, angle
        )

        self.avidrone.simple_goto(targetLocation)

        loop_count = 0
        while (
            self.avidrone.mode.name == "GUIDED"
        ):  # Stop action if we are no longer in guided mode.
            # print "DEBUG: mode: %s" % vehicle.mode.name
            remainingDistance = self.better_get_distance_meters(
                self.avidrone.location.global_frame, targetLocation
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
        next_waypoint = self.avidrone.commands.next
        if next_waypoint == 40:
            self.avidrone.mode = VehicleMode("GUIDED")
            print("breaking...")
            return True
        return False

    def condition_yaw(self, heading, relative):
        if relative:
            is_relative = True  # yaw relative to direction of travel
        else:
            is_relative = False  # yaw is an absolute angle

        if heading < 0:
            heading = abs(heading)
            cw = -1
        else:
            cw = 1

        msg = self.avidrone.message_factory.command_long_encode(
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
        self.avidrone.send_mavlink(msg)  # send command to vehicle

    def forward_calculation(self):
        flight_direction = []
        yaw = drone.vehicle.attitude.yaw

        print(yaw)
        flight_direction.append(MAGNITUDE * math.cos(yaw))
        flight_direction.append(MAGNITUDE * math.sin(yaw))

        return flight_direction

    def simple_goto_wait(self, go_to_checkpoint):
        self.avidrone.simple_goto(go_to_checkpoint)
        global_frame = self.avidrone.location.global_frame
        distance = get_distance_metres(
            Search.get_global_pos(global_frame), go_to_checkpoint
        )

        while distance >= DISTANCE_ERROR and self.avidrone.mode.name == "GUIDED":
            print(distance)
            distance = get_distance_metres(
                Search.get_global_pos(global_frame), go_to_checkpoint
            )
            time.sleep(1)

        if self.avidrone.mode.name != "GUIDED":
            self.avidrone.simple_goto(self.avidrone.location.global_frame)
            print("Halting simple_go_to")

        print("Checkpoint reached")

    def takeoff_to_altitude(self):
        self.avidrone.simple_takeoff(ALTITUDE)
        log.info(f"-- Taking off to altitude (m): {ALTITUDE} \n")

        while True:
            current_alt = self.avidrone.location.global_relative_frame.alt
            if current_alt >= ALTITUDE * 0.95:
                log.info(f"-- Reached {ALTITUDE}m")
                break
            time.sleep(1)

# TODO Remove this comment: implement
    def return_to_launch(self):
        pass

# TODO Remove this comment: implement
    def download_mission(self):
        pass
    
# TODO Remove this comment: implement
    def save_mission(self):
        pass

# TODO Remove this comment: implement
    def save_to_file(self, file, alt, width, d_len, len):
        AVIDRONE.commands.add(
        Command(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            ALTITUDE,
            )
        )

        if alt == 0:
            # TODO implement rectangular()
            Primary(Search).rectangular()
        else:
            # TODO implement rectangular_with_alt()
            Primary(Search).rectangular_with_alt()

        self.save_mission(file)
        AVIDRONE.commands.clear()

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
