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
from re import S

import drone
import dronekit_sitl
import gps_data
import transceiver.util
from dronekit import LocationGlobal, VehicleMode, connect
from transceiver.transceiver import Transceiver
from util import (ALTITUDE, DEGREES, LAND_THRESHOLD, MAGNITUDE, WINDOW_SIZE,
                  Mission, Search, get_distance_metres, get_location_metres,
                  get_location_metres_with_alt, get_range)

log = logging.getLogger(__name__)
log.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
file_handler = logging.FileHandler("secondary_.log")
file_handler.setFormatter(formatter)
log.addHandler(file_handler)

# Initialization
avidrone = drone.vehicle
mission = drone.mission
search = drone.search
sitl = drone.sitl
vector = drone.vector

beacon = search.mock_transceiver  # This is what is being searched for (rn)

# CLI conditions
IS_TEST = True
IS_VERBOSE = False

if IS_VERBOSE:
    log.info(f"- SEARCH PARAMETERS")
    log.info(f"-- altitude: {ALTITUDE}")
    log.info(f"-- degrees: {DEGREES}")
    log.info(f"-- land threshold: {LAND_THRESHOLD}")
    log.info(f"-- magnitude: {MAGNITUDE}")
    log.info(f"-- window size: {WINDOW_SIZE}")


def run(beacon):
    # Initialize values
    SIGNAL_FOUND = False
    theta = 90  # TODO replace with transceiver theta values
    uav_pos = [0, 0, 0]

    IS_TIMEOUT = False
    timeout_counter = 0

    if IS_TEST:
        beacon = search.mock_transceiver(uav_pos, beacon.position)
    log.info("----- secondary search ran successfully -----")

    while avidrone.mode.name == "GUIDED":
        gps_window = gps_data.GPSData(WINDOW_SIZE)
        if IS_TIMEOUT:  # return to landing
            log.critical("Return to launch site")
            avidrone.mode = VehicleMode("RTL")

        if (
            gps_window.get_minimum_index() == ((gps_window.window_size - 1) / 2)
            and len(gps_window.gps_points) == gps_window.window_size
        ):  # TODO add comment to explain what this means
            if transceiver.util.get_direction(theta) < 2:  # Turn left
                mission.condition_yaw(-DEGREES, True)

            elif transceiver.util.get_direction(theta) > 2:  # turn right
                mission.condition_yaw(DEGREES, True)

            elif transceiver.util.get_direction(theta) == 2:  # keep straight
                print("keep flying straight")
                gps_window.add_point(search.get_global_pos(), beacon.distance)

                mission.simple_goto_wait(
                    gps_window.gps_points[int((gps_window.window_size - 1) / 2)]
                )

                if gps_window.distance[2] <= LAND_THRESHOLD:
                    log.info("-- Landing")
                    avidrone.mode = VehicleMode("LAND")
                    SIGNAL_FOUND = True

                if SIGNAL_FOUND:
                    current_time = datetime.datetime.now()
                    log.info(
                        f"\n-------- VICTIM FOUND: {transceiver.signal_detected} -------- "
                    )
                    log.info(f"-- Time: {current_time}")
                    log.info(f"-- Location: {avidrone.location.global_frame}\n")

                else:
                    gps_window.purge_gps_window()

            elif (
                gps_window.get_minimum_index() == (gps_window.window_size - 1)
                and len(gps_window.gps_points) == gps_window.window_size
            ):  # If the minimum data point is the last one in the array,

                mission.condition_yaw(180, True)
                mission.simple_goto_wait(
                    gps_window.gps_points[gps_window.window_size - 1]
                )
                gps_window.purge_gps_window()

            elif gps_window.get_minimum_index() == 0:
                # If the minimum data point is in the first index,
                log.info("continue forward")
                mission.go_to_location(MAGNITUDE, avidrone.attitude.yaw, avidrone)

            else:
                timeout_counter += 1
                mission.go_to_location(MAGNITUDE, avidrone.attitude.yaw, avidrone)

        if timeout_counter == 100:
            IS_TIMEOUT = True
        time.sleep(2)


if __name__ == "__main__":
    uav_pos = [20, 200, 2]  # Example
    beacon_pos = [1, 1, 1]  # Example
    mock_transceiver = Search.mock_transceiver(uav_pos, beacon_pos)
    run(mock_transceiver)
