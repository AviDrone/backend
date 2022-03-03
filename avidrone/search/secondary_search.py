#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import datetime
import math
import time

from dronekit import LocationGlobal, VehicleMode, connect
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

import default_parameters as default
import read_transceiver
from gps_data import GPSData
from mission_methods import Search


def run():
    print("-- SECONDARY SEARCH --")
    parser = argparse.ArgumentParser(description="Demonstrates basic mission operations.")
    parser.add_argument(
        "--connect",
        help="vehicle connection target string. If not specified, SITL automatically started and used.",
    )
    args = parser.parse_args()

    connection_string = args.connect
    sitl = None

    # Start SITL if no connection string specified
    if not connection_string:
        import dronekit_sitl

        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    # Connect to the vehicle
    print("Connecting to vehicle on: %s" % connection_string)
    vehicle = connect(connection_string, wait_ready=True)

    signal_found = False
    Search.start()

    while vehicle.mode.name == "GUIDED":
        transceiver = Search.read_transceiver()

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

                Search.simple_goto_wait(
                    gps_window.gps_points[int((gps_window.window_size - 1) / 2)]
                )

                if gps_window.distance[2] <= default.LAND_THRESHOLD:
                    print("-- Landing")
                    vehicle.mode = VehicleMode("LAND")
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
                go_to_location(default.MAGNITUDE, vehicle.attitude.yaw, vehicle)


            else:
                print(f"Did not find signal at altitude: {default.ALTITUDE}")
                print("Climbing...")
                go_to_location(default.MAGNITUDE, vehicle.attitude.yaw, vehicle)

        time.sleep(2)

if __name__ == "__main__":
    run()
