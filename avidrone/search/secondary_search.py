#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import datetime
import math
import time

import default_parameters as default
import read_transceiver
from dronekit import LocationGlobal, VehicleMode, connect
from gps_data import GPSData
from pymavlink import mavutil

print(f"-- default size: {default.DEGREES}")
print(f"-- default altitude: {default.ALTITUDE}")
print(f"-- default land threshold: {default.LAND_THRESHOLD} \n")


parser = argparse.ArgumentParser(description="Demonstrates basic mission operations.")
parser.add_argument(
    "--connect",
    help="drone connection target string. If not specified, SITL automatically started and used.",
)
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl

    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the drone
print("Connecting to drone on: %s" % connection_string)
drone = connect(connection_string, wait_ready=True)


def get_global_pos():
    return drone.location.global_frame


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
                + math.cos(lat1) * math.cos(lat2) * (math.sin((lon1 - lon2) / 2)) ** 2
            )
        )
        * 1.113195e5
    )
    return distance


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


def better_goto(distance, angle, drone):
    """
    Moves the drone to a position d_north metres North and d_east metres East of the current position.
    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for
    the target position. This allows it to be called with different position-setting commands.
    By default it uses the standard method: dronekit.lib.vehicle.simple_goto().
    The method reports the distance to target every two seconds.

    Retrieved from this link:
    https://dronekit-python.readthedocs.io/en/latest/examples/guided-set-speed-yaw-demo.html
    """

    currentLocation = drone.location.global_frame  # was global_relative_frame
    targetLocation = better_get_location_meters(currentLocation, distance, angle)
    # targetDistance = better_get_distance_meters(currentLocation, targetLocation)
    # targetDistance = get_distance_meters(currentLocation, targetLocation)
    drone.simple_goto(targetLocation)

    # print "DEBUG: targetLocation: %s" % targetLocation
    # print "DEBUG: targetLocation: %s" % targetDistance

    loop_count = 0
    while (
        drone.mode.name == "GUIDED"
    ):  # Stop action if we are no longer in guided mode.
        # print "DEBUG: mode: %s" % drone.mode.name
        remainingDistance = better_get_distance_meters(
            drone.location.global_frame, targetLocation
        )
        # global_frame was global_relative_frame
        # remainingDistance=get_distance_meters(drone.location.global_frame, targetLocation)
        # global_frame was global_relative_frame
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


def simple_goto_wait(goto_checkpoint):
    drone.simple_goto(goto_checkpoint)

    distance = better_get_distance_meters(get_global_pos(), goto_checkpoint)

    while distance >= default.DISTANCE_ERROR and drone.mode.name == "GUIDED":
        print(distance)
        distance = better_get_distance_meters(get_global_pos(), goto_checkpoint)
        time.sleep(1)

    if drone.mode.name != "GUIDED":
        drone.simple_goto(drone.location.global_frame)
        print("Halting simple_goto")

    print("Checkpoint reached")


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
    msg = drone.message_factory.command_long_encode(
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
    # send command to drone
    drone.send_mavlink(msg)

    # make drone wait until the wobbling settles down before moving forward.
    heading_rad = heading * math.pi / 180
    target_yaw = original_yaw + cw * heading_rad

    while (
        abs(target_yaw - drone.attitude.yaw) % math.pi > 0.01745 * default.DEGREE_ERROR
    ):
        print(
            "Turn error: ", abs(target_yaw - drone.attitude.yaw) % math.pi
        )  # 1 Degree of error....
        time.sleep(0.25)


def takeoff_to(default):
    target_altitude = default.ALTITUDE
    print(f"-- Taking off to altitude (m): {default.ALTITUDE} \n")
    drone.simple_takeoff(target_altitude)

    while True:
        if drone.location.global_relative_frame.alt >= target_altitude * 0.95:
            print(f"-- Reached {default.ALTITUDE}m")
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
        takeoff_to(default.ALTITUDE)


print("-- Initializing the gps window")  # to be default.WINDOW_SIZE long
gps_window = GPSData(default.WINDOW_SIZE)

print("-- Setting GUIDED flight mode")
print("-- Waiting for GUIDED mode...")

while drone.mode.name != "GUIDED":
    time.sleep(1)


def secondary_search() -> None:
    print("-- SECONDARY SEARCH --")
    signal_found = False
    initialize()  # uav

    while drone.mode.name == "GUIDED":
        transceiver = read_transceiver()  # TODO implement read_transceiver
        print(transceiver.direction, ", ", transceiver.distance)

        if transceiver.direction < 2:  # Turn left
            print("-- Turning left")
            condition_yaw(-default.DEGREES, True)

        elif transceiver.direction > 2:  # Turn right
            print("-- Turning right")
            condition_yaw(default.DEGREES, True)

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

                if gps_window.distance[2] <= default.LAND_THRESHOLD:
                    print("-- Landing")
                    drone.mode = VehicleMode("LAND")
                    signal_found = True

                if signal_found:
                    current_time = datetime.datetime.now()
                    print("--- SIGNAL FOUND --- ", f"-- time: {current_time}")

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
                better_goto(default.MAGNITUDE, drone.attitude.yaw, drone)

            else:
                print(f"Did not find signal at altitude: {default.ALTITUDE}")
                print("Climbing...")
                better_goto(default.MAGNITUDE, drone.attitude.yaw, drone)
        time.sleep(2)


secondary_search()
