#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import print_function

import asyncio
import datetime
import pathlib
import time
import serial

from dronekit import (LocationGlobal, LocationGlobalRelative,
                      VehicleMode, connect)
from pymavlink import mavutil
from transceiver.direction_distance import DirectionDistance as transceiver
from UAV import default_parameters

default = default_parameters.parameter()

default_degrees = default[2]
default_altitude = default[4]
default_land_threshold = default[5]

drone = connect("/dev/serial0", baud=57600)
print("-- Waiting for drone to connect...")
vehicle.wait_ready("-- Drone Found!")


def arm() -> None:
    print("-- Waiting for drone to initialize...")
    while not drone.is_armable:
        time.sleep(1)

    drone.mode = VehicleMode("GUIDED")
    drone.armed = True

    print("-- Arming...")
    while not vehicle.is_armable:
        time.sleep(1)


def takeoff_to(default_altitude):
    print(f"-- Taking off to altitude (m): {default_altitude} \n")
    drone.simple_takeoff(target_altitude)

    while True:
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print(f"-- Reached {default_altitude}m")
            break
        time.sleep(1)


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

    # create the CONDITION_YAW command using command_long_encode()
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
    # send command to vehicle
    vehicle.send_mavlink(msg)

    """
    Adding additional code to make drone wait till the wobbling settles down
    before moving forward.
    """
    heading_rad = heading * math.pi / 180
    target_yaw = original_yaw + cw * heading_rad

    # 1 Degree of error....
    while abs(target_yaw - vehicle.attitude.yaw) % math.pi > 0.01745 * DEGREE_ERROR:
        print("Turn error: ", abs(target_yaw - vehicle.attitude.yaw) % math.pi)
        time.sleep(0.25)


def initialize():
    arm()
    if drone.armed:
        print(f"-- Armed: {drone.armed}")
        takeoff_to(default_altitude)


print("-- Initializing the gps_window")  # to be default_window_size long
gps_window = GPSData(default_window_size)

print("-- Setting GUIDED flight mode")
while vehicle.mode.name != "GUIDED":
    print("Waiting for GUIDED mode")
    time.sleep(1)


def secondary_search() -> None:
    signal_found = False
    print("\n -- SECONDARY SEARCH -- \n")
    initialize()  # UAV

    print("-- transceiver reading (direction, distance):")
    while vehicle.mode.name == "GUIDED":
        transceiver = read_transceiver()
        print(transceiver.direction, ", ", transceiver.distance)

        if transceiver.direction < 2:  # Turn left
            print("-- Turning left")
            condition_yaw(-DEGREES, True)

        elif transceiver.direction > 2:  # Turn right
            print("-- Turning right")
            condition_yaw(DEGREES, True)

        elif transceiver.direction == 2:  # Continue forward
            print("-- Continuing forward")
            gps_window.add_point(get_global_pos(), direction_distance.distance)
            if (
                gps_window.get_minimum_index() == ((gps_window.window_size - 1) / 2)
                and len(gps_window.gps_points) == gps_window.window_size
            ):

                # If the minimum is the center point of the gps_window we need to go
                # back to that location, Min index = middle

                simple_goto_wait(
                    gps_window.gps_points[int((gps_window.window_size - 1) / 2)]
                )

                if gps_window.distance[2] <= default_land_threshold:
                    print("-- Landing")
                    vehicle.mode = VehicleMode("LAND")
                    signal_found = True

                if signal_found:
                    current_time = datetime.datetime.now()
                    print("--- SIGNAL FOUND --- ", f"-- time: {current_time}")
                    print(f"-- location: {global_position}")

                else:
                    print("Not close, continuing")
                    gps_window.purge_gps_window()

            elif (
                gps_window.get_minimum_index() == (gps_window.window_size - 1)
                and len(gps_window.gps_points) == gps_window.window_size
            ):

                # If the minimum data point is the last one in the array,
                print("too far in the wrong direction")

                condition_yaw(180, True)
                simple_goto_wait(gps_window.gps_points[gps_window.window_size - 1])
                gps_window.purge_gps_window()

            elif gps_window.get_minimum_index() == 0:
                # If the minimum data point is in the first index,
                print("continue forward")
                better_goto(MAGNITUDE, vehicle.attitude.yaw, vehicle)

            else:
                print(f"Did not find signal at altitude: {default_altitude}")
                print("Climbing...")
                better_goto(MAGNITUDE, vehicle.attitude.yaw, vehicle)
        time.sleep(2)


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(initialize())
    secondary_search()
