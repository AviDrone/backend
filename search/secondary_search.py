#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import time
import datetime
import default_parameters
import read_transceiver
from dronekit import VehicleMode, connect

from transceiver_output import DirectionDistance
from gps_data import GPSData

default = default_parameters.parameter()
default_size = len(default)

default_magnitude = default[0]
default_altitude = default[1]
default_degrees = default[2]
default_degree_error = default[3]
default_distance_error = default[4]
default_land_threshold = default[5]
default_window_size = default[6]

print(f"-- default size: {default_degrees}")
print(f"-- default altitude: {default_altitude}")
print(f"-- default land threshold: {default_land_threshold} \n")


parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect',
                    help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl

    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
drone = connect(connection_string, wait_ready=True)


def condition_yaw(heading, relative=False):
    """
    Modified to allow for clockwise and counter-clockwise operation
    """
    original_yaw = drone.attitude.yaw
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
    drone.send_mavlink(msg)

    # make drone wait until the wobbling settles down before moving forward.
    heading_rad = heading * math.pi / 180
    target_yaw = original_yaw + cw * heading_rad

    while abs(target_yaw - vehicle.attitude.yaw) % math.pi > 0.01745 * default_degree_error:
        print("Turn error: ", abs(target_yaw - vehicle.attitude.yaw) % math.pi)  # 1 Degree of error....
        time.sleep(0.25)


def takeoff_to(default_altitude):
    print(f"-- Taking off to altitude (m): {default_altitude} \n")
    drone.simple_takeoff(target_altitude)

    while True:
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print(f"-- Reached {default_altitude}m")
            break
        time.sleep(1)


def initialize():
    print("-- Waiting for drone to initialize...")
    while not drone.is_armable:
        time.sleep(1)

    drone.mode = VehicleMode("GUIDED")
    drone.armed = True

    print("-- Arming...")
    while not drone.is_armable:
        time.sleep(1)

    if drone.armed:
        print(f"-- Armed: {drone.armed}")
        takeoff_to(default_altitude)


print("-- Initializing the gps window")  # to be default_window_size long
gps_window = GPSData(default_window_size)

print("-- Setting GUIDED flight mode")
print("-- Waiting for GUIDED mode...")

while drone.mode.name != "GUIDED":
    time.sleep(1)


def secondary_search() -> None:
    print("-- SECONDARY SEARCH --")
    signal_found = False
    initialize()  # UAV

    while vehicle.mode.name == "GUIDED":
        transceiver = read_transceiver()
        print(transceiver.direction, ", ", transceiver.distance)

        if transceiver.direction < 2:  # Turn left
            print("-- Turning left")
            condition_yaw(-default_degrees, True)

        elif transceiver.direction > 2:  # Turn right
            print("-- Turning right")
            condition_yaw(default_degrees, True)

        elif transceiver.direction == 2:  # Continue forward
            print("-- Continuing forward")
            gps_window.add_point(get_global_pos(), transceiver.distance)
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
                    drone.mode = VehicleMode("LAND")
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
                better_goto(default_magnitude, drone.attitude.yaw, vehicle)

            else:
                print(f"Did not find signal at altitude: {default_altitude}")
                print("Climbing...")
                better_goto(default_magnitude, drone.attitude.yaw, vehicle)
        time.sleep(2)


secondary_search()
