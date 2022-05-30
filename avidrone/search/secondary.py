#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    SECONDARY SEARCH
"""
from __future__ import print_function

# Set up option parsing to get connection string
import argparse
import datetime
import logging
import math
import time

import drone
import dronekit_sitl
import transceiver
import transceiver.util
from dronekit import LocationGlobal, VehicleMode, connect
from util import (
    ALTITUDE,
    DEGREES,
    LAND_THRESHOLD,
    MAGNITUDE,
    WINDOW_SIZE,
    Mission,
    Search,
    get_distance_metres,
    get_location_metres,
    get_location_metres_with_alt,
    get_range
)

log = logging.getLogger(__name__)
log.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s:%(levelname)s:%(message)s")
file_handler = logging.FileHandler("secondary.log")
file_handler.setFormatter(formatter)
log.addHandler(file_handler)

# initialization
aviDrone = drone.vehicle
sitl = drone.sitl
vector = drone.vector
mission = drone.mission
search = drone.search

# If test, use the following command to run
IS_TEST = True  # set to False for real flight

# TODO add timeout like with transceiver

"""
            log.info("Return to launch")
            Avidrone.mode = VehicleModle("RTL)")
"""

def run(transceiver):
    log.info("-- SECONDARY SEARCH --")
    signal_found = False
    uav_pos = [0, 0, 0]
    beacon_pos = [0, 0, 0]
    if IS_TEST:
        beacon = search.mock_transceiver(uav_pos, beacon_pos)  # mock transceiver
    else:
        beacon = transceiver.Transceiver()
    search.start()
    gps_window = WINDOW_SIZE
    while aviDrone.mode.name == "GUIDED":
        log.info(transceiver.direction, ", ", transceiver.distance)

        if beacon.direction < 2:  # Turn left
            log.info("-- Turning left")
            mission.condition_yaw(-DEGREES, True)

        elif beacon.direction > 2:  # Turn right
            log.info("-- Turning right")
            mission.condition_yaw(DEGREES, True)

        elif beacon.direction == 2:  # Continue forward
            log.info("-- Continuing forward")
            gps_window.add_point(search.get_global_pos(), transceiver.distance)
            if (
                gps_window.get_minimum_index() == ((gps_window.window_size - 1) / 2)
                and len(gps_window.gps_points) == gps_window.window_size
            ):

                # If the minimum is the center point of the gps_window we need to go
                # back to that location, Min index = middle

                mission.simple_goto_wait(
                    gps_window.gps_points[int((gps_window.window_size - 1) / 2)]
                )

                if gps_window.distance[2] <= LAND_THRESHOLD:
                    log.info("-- Landing")
                    aviDrone.mode = VehicleMode("LAND")
                    signal_found = True

                if signal_found:
                    current_time = datetime.datetime.now()
                    print(
                        f"\n-------- VICTIM FOUND: {transceiver.signal_detected} -------- "
                    )
                    print(f"-- Time: {current_time}")
                    print(f"-- Location: {aviDrone.location.global_frame}\n")

                else:
                    log.info("Not close, continuing")
                    gps_window.purge_gps_window()

            elif (
                gps_window.get_minimum_index() == (gps_window.window_size - 1)
                and len(gps_window.gps_points) == gps_window.window_size
            ):

                # If the minimum data point is the last one in the array,
                log.info("too far in the wrong direction")

                mission.condition_yaw(180, True)
                mission.simple_goto_wait(
                    gps_window.gps_points[gps_window.window_size - 1]
                )
                gps_window.purge_gps_window()

            elif gps_window.get_minimum_index() == 0:
                # If the minimum data point is in the first index,
                log.info("continue forward")
                mission.go_to_location(MAGNITUDE, aviDrone.attitude.yaw, aviDrone)

            else:
                log.info(f"Did not find signal at altitude: {ALTITUDE}")
                log.info("Climbing...")
                mission.go_to_location(MAGNITUDE, aviDrone.attitude.yaw, aviDrone)
        time.sleep(2)


# A version much closer to the previous version to serve as a better reference
def run_prev_sec_search():
    # Initialize the gps_window to be WINDOW_SIZE long
    import gps_data
    import transceiver.read_transceiver as rt
    gps_window = gps_data.GPSData(WINDOW_SIZE)

    while aviDrone.mode.name != "GUIDED":
        print("Waiting for GUIDED mode")
        time.sleep(1)

    while aviDrone.mode.name == "GUIDED":
        direction_distance = rt.read_transceiver()

        print("Direction: ", direction_distance.direction, "Distance: ", direction_distance.distance)

        if direction_distance.direction < 2:
            # Turn left
            mission.condition_yaw(-DEGREES, True)

        elif direction_distance.direction > 2:
            # Turn right
            mission.condition_yaw(DEGREES, True)

        elif direction_distance.direction == 2:
            print("Fly forward")
            flight_dir = mission.forward_calculation()
            print("Flight dir[0]: ", flight_dir[0])
            print("Flight dir[1]: ", flight_dir[1])

            gps_window.add_point(search.get_global_pos(), direction_distance.distance)

            if gps_window.get_minimum_index() == ((gps_window.window_size - 1) / 2) \
                    and len(gps_window.gps_points) == gps_window.window_size:

                # If the minimum is the center point of the gps_window we need to go
                # back to that location
                print("Min index = middle")

                mission.simple_goto_wait(gps_window.gps_points[int((gps_window.window_size - 1) / 2)])

                if gps_window.distance[2] <= LAND_THRESHOLD:
                    print("Close, landing")
                    aviDrone.mode = VehicleMode('LAND')
                else:
                    print("Not close, continuing")
                    gps_window.purge_gps_window()

            elif gps_window.get_minimum_index() == (gps_window.window_size - 1) \
                    and len(gps_window.gps_points) == gps_window.window_size:

                # If the minimum data point is the last one in the array we have gone
                # too far and in the wrong direction
                print("Min index = window_size - 1")

                mission.condition_yaw(180, True)
                mission.simple_goto_wait(gps_window.gps_points[gps_window.window_size - 1])
                gps_window.purge_gps_window()

            elif gps_window.get_minimum_index() == 0:
                # If the minimum data point is in the first index, continue forward
                print("Min index = 0")

                # goto(flight_dir[0], flight_dir[1])
                search.better_goto(MAGNITUDE, aviDrone.attitude.yaw, aviDrone)

            else:
                # Possibly going in the wrong direction.... But we still need to keep
                # going to make sure

                print("Goin' on up")
                # goto(flight_dir[0], flight_dir[1])
                mission.better_goto(MAGNITUDE, aviDrone.attitude.yaw, aviDrone)

            # send_ned_velocity(flight_dir[0], flight_dir[1], 0, 1)

        time.sleep(2)

if __name__ == "__main__":
    uav_pos = [20, 20, 2]  # Example
    beacon_pos = [1, 1, 1]  # Example
    search = drone.search
    mock_transceiver = Search.mock_transceiver(uav_pos, beacon_pos)
    run(mock_transceiver)
