#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    SECONDARY SEARCH
"""
from __future__ import print_function

# Set up option parsing to get connection string
import argparse
import ctypes
import datetime
import logging
import math
import pathlib
import time
from re import S

import drone
import dronekit_sitl
import transceiver.util
from dronekit import LocationGlobal, VehicleMode
from transceiver import EM_field, util
from transceiver.transceiver import Transceiver
from util import (
    ALTITUDE,
    DEGREES,
    LAND_THRESHOLD,
    MAGNITUDE,
    WINDOW_SIZE,
    GpsData,
    Mission,
    Search,
)

log = logging.getLogger(__name__)
log.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
file_handler = logging.FileHandler("secondary.log")
file_handler.setFormatter(formatter)
log.addHandler(file_handler)

log.info("**************** SECONDARY SEARCH ****************")
# Initialization
avidrone = drone.vehicle
mission = drone.mission
search = drone.search
sitl = drone.sitl
vector = drone.vector

# CLI conditions
IS_TEST = True
IS_VERBOSE = False

if IS_VERBOSE:
    log.debug(f"Verbose: {IS_VERBOSE}")
    log.info(f"- SEARCH PARAMETERS")
    log.info(f"-- altitude: {ALTITUDE}")
    log.info(f"-- land threshold: {LAND_THRESHOLD}")


def run(beacon):
    IS_TIMEOUT = False
    timeout_counter = 0

    # TODO write handoff sequence (from state model)

    # Initialize values
    SIGNAL_FOUND = False
    uav_pos = [
        avidrone.location.global_frame.lat,
        avidrone.location.global_frame.lon,
        avidrone.location.global_frame.alt,
    ]

    gps_window = GpsData(WINDOW_SIZE)

    if IS_TEST:
        mock_EM_field = EM_field.EM_field()
        mock_theta = mock_EM_field.get_theta_at_pos(uav_pos)
        beacon = search.mock_transceiver(beacon_pos, uav_pos)

    else:
        beacon = Transceiver.read_transceiver()

    # Confirm we are in GUIDED mode
    while avidrone.mode.name != "GUIDED":
        avidrone.mode = VehicleMode("GUIDED")
        log.info("Waiting for GUIDED mode...")
        time.sleep(1)

    while avidrone.mode.name == "GUIDED":
        if IS_TIMEOUT:  # return to landing
            log.critical("Reached timeout. Returning to launch site.")
            avidrone.mode = VehicleMode("RTL")

        if transceiver.util.get_direction(mock_theta) < 2.0:  # Turn left
            mission.condition_yaw(-DEGREES, True)

        elif transceiver.util.get_direction(mock_theta) > 2.0:  # Turn right
            mission.condition_yaw(DEGREES, True)

        elif transceiver.util.get_direction(mock_theta) == 2.0:  # Keep straight
            gps_window.add_point(search.get_global_pos(), beacon.distance)

        # If the minimum is the center point of the gps_window,
        # we need to go back to that location
        if (
            gps_window.get_minimum_index() == ((gps_window.window_size - 1) / 2)
            and len(gps_window.gps_points) == gps_window.window_size
        ):
            mission.simple_goto_wait(
                gps_window.gps_points[int((gps_window.window_size - 1) / 2)]
            )

            if gps_window.distance[2] <= LAND_THRESHOLD:
                SIGNAL_FOUND = True

            if SIGNAL_FOUND:
                avidrone.mode = VehicleMode("LAND")
                log.warning("-- Landing")
                current_time = datetime.datetime.now()
                beacon.signal_found_msg()
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
        ):
            # If the minimum data point is the last one in the array we have gone
            # too far and in the wrong direction
            mission.condition_yaw(180, True)
            mission.simple_goto_wait(gps_window.gps_points[gps_window.window_size - 1])
            gps_window.purge_gps_window()

        elif gps_window.get_minimum_index() == 0:
            # If the minimum data point is in the first index,
            log.info("continue forward")
            mission.better_goto(MAGNITUDE, avidrone.attitude.yaw)

        else:
            timeout_counter += 1
            mission.better_goto(MAGNITUDE, avidrone.attitude.yaw)

        if timeout_counter == 100:
            IS_TIMEOUT = True
        time.sleep(2)


if __name__ == "__main__":

    # shared_ctypes_lib = pathlib.Path().absolute() / "read_transceiver.lib"    # TODO create read_transceiver
    # c_lib = ctypes.CDLL(shared_ctypes_lib)

    uav_pos = [20, 200, 2]  # Example
    beacon_pos = [1, 1, 1]  # Example
    mock_transceiver = Search.mock_transceiver(uav_pos, beacon_pos)
    run(mock_transceiver)
    log.info("----- secondary search ran successfully -----")
