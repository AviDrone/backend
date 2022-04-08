#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import asyncio
import datetime
import math
import numpy as np
import time

import transceiver.utils as util
import default_parameters as default
from dronekit import (
    Command,
    LocationGlobal,
    LocationGlobalRelative,
    VehicleMode,
    connect,
)
from gps_data import GPSData


class Search:
    def __init__(self):
        global_frame = vehicle.location.global_frame
        global_location = LocationGlobal(new_lat, new_lon, original_location.alt)
        distance = (2 * math.asin(math.sqrt(
            (math.sin((lat_a - lat_b) / 2)
             ) ** 2 + math.cos(lat_a) * math.cos(lat_b) *
            (math.sin((lon_a - lon_b) / 2)) ** 2)) * 1.113195e5)

    @staticmethod
    def get_location(self, original_location, d_north, d_east):
        earth_radius = 6378137.0  # Radius of "spherical" earth
        # Coordinate offsets in radians
        d_lat = d_north / earth_radius
        d_lon = d_east / (
                earth_radius * math.cos(math.pi * original_location.lat / 180)
        )

        # New position in decimal degrees
        new_lat = original_location.lat + (d_lat * 180 / math.pi)
        new_lon = original_location.lon + (d_lon * 180 / math.pi)

        return global_location

    @staticmethod
    def get_distance(self, location_a, location_b):
        lat_a, lat_b = location_a.lat, location_b.lat
        lon_a, lon_b = location_a.lat, location_b.lat

        return distance

    @staticmethod
    def get_global_pos(self):
        return global_frame

    @staticmethod
    def read_transceiver(self):
        uav_pos = [2, 2, 2]  # TODO replace this with actual positions
        beacon_pos = [1, 1, 1]  # TODO replace this with actual positions
        return util.mock_beacon(uav_pos, beacon_pos)


def start_gps():
    print("-- Initializing the gps window")  # to be default.WINDOW_SIZE long
    gps_window = GPSData(default.WINDOW_SIZE)


def start():
    print("-- Waiting for vehicle to start...")
    while not vehicle.is_armable:
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    print("-- Arming...")
    while not vehicle.is_armable:
        time.sleep(1)

    if vehicle.armed:
        print(f"-- Armed: {vehicle.armed}")
        takeoff_to(default.ALTITUDE)
    start_gps()

    print("-- Setting GUIDED flight mode")
    print("-- Waiting for GUIDED mode...")

    while vehicle.mode.name != "GUIDED":
        time.sleep(1)


def takeoff_to(default):
    target_altitude = default.ALTITUDE
    print(f"-- Taking off to altitude (m): {default.ALTITUDE} \n")
    vehicle.simple_takeoff(target_altitude)

    while True:
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print(f"-- Reached {default.ALTITUDE}m")
            break
        time.sleep(1)


def go_to_location(distance, angle, vehicle):
    current_location = vehicle.location.global_frame
    target_location = get_location(current_location, distance, angle)
    vehicle.simple_goto(target_location)

    loop_count = 0
    while vehicle.mode.name == "GUIDED":
        remaining_distance = get_distance(
            vehicle.location.global_frame, target_location
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

    def simple_goto_wait(goto_checkpoint):
        vehicle.simple_goto(goto_checkpoint)

        distance = better_get_distance_meters(get_global_pos(), goto_checkpoint)

        while distance >= default.DISTANCE_ERROR and vehicle.mode.name == "GUIDED":
            print(distance)
            distance = better_get_distance_meters(get_global_pos(), goto_checkpoint)
            time.sleep(1)

        if vehicle.mode.name != "GUIDED":
            vehicle.simple_goto(vehicle.location.global_frame)
            print("Halting simple_goto")

        print("Checkpoint reached")


def wobble_wait():
    # make vehicle wait until the wobbling settles down before moving forward.
    heading_rad = heading * math.pi / 180

    target_yaw = original_yaw + cw * heading_rad

    while (
            abs(target_yaw - vehicle.attitude.yaw) % math.pi
            > 0.01745 * default.DEGREE_ERROR
    ):
        error_degree = abs(target_yaw - vehicle.attitude.yaw) % math.pi
        print("Turn error: ", error_degree)  # 1 degree
        time.sleep(0.25)


def condition_yaw(heading, relative=False):
    """
    Modified to allow for clockwise and counter-clockwise operation
    """
    original_yaw = vehicle.attitude.yaw
    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle

    if heading < 0:
        heading = abs(heading)
        cw = -1
    else:
        cw = 1

    msg = vehicle.message_factory.command_long_encode(
        0,  # target system
        0,  # target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading,  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s
        cw,  # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0,
        0,
        0,
    )  # param 5 ~ 7 not used
    vehicle.send_mavlink(msg)  # send command to vehicle
    wobble_wait()


def Rotate_Cloud(Points, V1, V2):
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
    V1V2CrossNorm = (V1V2Cross[0] ** 2 + V1V2Cross[1] ** 2 + V1V2Cross[2] ** 2) ** 0.5
    V1V2CrossNormalized = V1V2Cross / V1V2CrossNorm

    # Dot product
    V1V2Dot = np.dot(V1, V2)
    V1Norm = (V1[0] ** 2 + V1[1] ** 2 + V1[2] ** 2) ** 0.5
    V2Norm = (V2[0] ** 2 + V2[1] ** 2 + V2[2] ** 2) ** 0.5

    # Angle between the vectors
    Theta = np.arccos(V1V2Dot / (V1Norm * V2Norm))
    print(Theta)

    # Using Rodrigues' rotation formula (wikipedia):
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


def Rotate_Vector(Vector, Angle):
    # Vector is the vector being rotated
    # Angle is used to rotate Vector and is given in degrees

    # Convert angle to radians
    Angle_Rad = np.radians(Angle)

    # rotation matrix
    # See https://en.wikipedia.org/wiki/Rotation_matrix for more information
    r = np.array(
        (
            (np.cos(Angle_Rad), -np.sin(Angle_Rad)),
            (np.sin(Angle_Rad), np.cos(Angle_Rad)),
        )
    )

    # we only care about x and y, not z
    a_vector = (Vector[0], Vector[1])
    a_vector = np.asarray(a_vector)

    # vector after rotation
    rotated = r.dot(a_vector)
    # print(rotated)

    # return 3D vector
    NewVector = (rotated[0], rotated[1], Vector[2])

    return NewVector
