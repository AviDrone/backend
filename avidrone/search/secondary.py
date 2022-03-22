#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import datetime
import logging as log
import math
import time

import default_parameters as default
import dronekit_sitl
from dronekit import LocationGlobal, VehicleMode, connect
from gps_data import GPSData
from mission_methods import Search
from transceiver import Transceiver


def run():
    print("-- SECONDARY SEARCH --")
    parser = argparse.ArgumentParser(
        description="Demonstrates basic mission operations."
    )
    parser.add_argument(
        "--connect",
        help="vehicle connection target string. If not specified, SITL automatically started and used.",
    )
    args = parser.parse_args()

    connection_string = args.connect

    # Start SITL if no connection string specified
    if not connection_string:
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    # Connect to the vehicle
    log.info("Connecting to vehicle on: %s", connection_string)
    vehicle = connect(connection_string, wait_ready=True)

    signal_found = False
    Search.start()
    gps_window = GPSData(default.WINDOW_SIZE)
    while vehicle.mode.name == "GUIDED":
        # TODO implement transceiver
        transceiver = Transceiver
        log.info(transceiver.direction, ", ", transceiver.distance)

        if transceiver.direction < 2:  # Turn left
            log.info("-- Turning left")
            Search.condition_yaw(-default.DEGREES, True)

        elif transceiver.direction > 2:  # Turn right
            log.info("-- Turning right")
            Search.condition_yaw(default.DEGREES, True)

        elif transceiver.direction == 2:  # Continue forward
            log.info("-- Continuing forward")
            gps_window.add_point(Search.get_global_pos(), transceiver.distance)
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
                    log.info("-- Landing")
                    vehicle.mode = VehicleMode("LAND")
                    signal_found = True

                if signal_found:
                    current_time = datetime.datetime.now()
                    print("--- SIGNAL FOUND --- ", f"-- time: {current_time}")

                else:
                    log.info("Not close, continuing")
                    gps_window.purge_gps_window()

            elif (
                gps_window.get_minimum_index() == (gps_window.window_size - 1)
                and len(gps_window.gps_points) == gps_window.window_size
            ):

                # If the minimum data point is the last one in the array,
                log.info("too far in the wrong direction")

                Search.condition_yaw(180, True)
                Search.simple_goto_wait(
                    gps_window.gps_points[gps_window.window_size - 1]
                )
                gps_window.purge_gps_window()

            elif gps_window.get_minimum_index() == 0:
                # If the minimum data point is in the first index,
                log.info("continue forward")
                Search.go_to_location(default.MAGNITUDE, vehicle.attitude.yaw, vehicle)

            else:
                log.info(f"Did not find signal at altitude: {default.ALTITUDE}")
                log.info("Climbing...")
                Search.go_to_location(default.MAGNITUDE, vehicle.attitude.yaw, vehicle)
        time.sleep(2)


if __name__ == "__main__":
    run()
