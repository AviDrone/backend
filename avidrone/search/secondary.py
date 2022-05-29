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
    get_range,
)

log = logging.getLogger(__name__)
log.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s:%(levelname)s:%(message)s")
file_handler = logging.FileHandler("secondary.log")
file_handler.setFormatter(formatter)
log.addHandler(file_handler)

# initialization
Avidrone = drone.vehicle
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
    while Avidrone.mode.name == "GUIDED":
        log.info(transceiver.direction, ", ", transceiver.distance)

        if beacon.direction < 2:  # Turn left
            log.info("-- Turning left")
            Mission.condition_yaw(-DEGREES, True)

        elif beacon.direction > 2:  # Turn right
            log.info("-- Turning right")
            Mission.condition_yaw(DEGREES, True)

        elif beacon.direction == 2:  # Continue forward
            log.info("-- Continuing forward")
            gps_window.add_point(search.get_global_pos(), transceiver.distance)
            if (
                gps_window.get_minimum_index() == ((gps_window.window_size - 1) / 2)
                and len(gps_window.gps_points) == gps_window.window_size
            ):

                # If the minimum is the center point of the gps_window we need to go
                # back to that location, Min index = middle

                Mission.simple_goto_wait(
                    gps_window.gps_points[int((gps_window.window_size - 1) / 2)]
                )

                if gps_window.distance[2] <= LAND_THRESHOLD:
                    log.info("-- Landing")
                    Avidrone.mode = VehicleMode("LAND")
                    signal_found = True

                if signal_found:
                    current_time = datetime.datetime.now()
                    print(
                        f"\n-------- VICTIM FOUND: {transceiver.signal_detected} -------- "
                    )
                    print(f"-- Time: {current_time}")
                    print(f"-- Location: {Avidrone.location.global_frame}\n")

                else:
                    log.info("Not close, continuing")
                    gps_window.purge_gps_window()

            elif (
                gps_window.get_minimum_index() == (gps_window.window_size - 1)
                and len(gps_window.gps_points) == gps_window.window_size
            ):

                # If the minimum data point is the last one in the array,
                log.info("too far in the wrong direction")

                Mission.condition_yaw(180, True)
                Mission.simple_goto_wait(
                    gps_window.gps_points[gps_window.window_size - 1]
                )
                gps_window.purge_gps_window()

            elif gps_window.get_minimum_index() == 0:
                # If the minimum data point is in the first index,
                log.info("continue forward")
                Mission.go_to_location(MAGNITUDE, Avidrone.attitude.yaw, Avidrone)

            else:
                log.info(f"Did not find signal at altitude: {ALTITUDE}")
                log.info("Climbing...")
                Mission.go_to_location(MAGNITUDE, Avidrone.attitude.yaw, Avidrone)
        time.sleep(2)


if __name__ == "__main__":
    uav_pos = [20, 20, 2]  # Example
    beacon_pos = [1, 1, 1]  # Example
    search = drone.search
    mock_transceiver = Search.mock_transceiver(uav_pos, beacon_pos)
    run(mock_transceiver)
