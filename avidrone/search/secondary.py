#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    SECONDARY SEARCH
"""
from __future__ import print_function

# Set up option parsing to get connection string
import argparse
import datetime
import logging as log
import math
import time
import drone
import dronekit_sitl
import util
from dronekit import LocationGlobal, VehicleMode, connect
from transceiver import Transceiver


AviDrone = drone.vehicle
def run():
    signal_found = False
    log.info("-- SECONDARY SEARCH --")
    util.Search.start()
    gps_window = util.WINDOW_SIZE
    while AviDrone.mode.name == "GUIDED":
        # TODO implement transceiver
        transceiver = Transceiver
        log.info(transceiver.direction, ", ", transceiver.distance)

        if transceiver.direction < 2:  # Turn left
            log.info("-- Turning left")
            util.Search.condition_yaw(-util.DEGREES, True)

        elif transceiver.direction > 2:  # Turn right
            log.info("-- Turning right")
            util.Search.condition_yaw(util.DEGREES, True)

        elif transceiver.direction == 2:  # Continue forward
            log.info("-- Continuing forward")
            gps_window.add_point(util.Search.get_global_pos(), transceiver.distance)
            if (
                gps_window.get_minimum_index() == ((gps_window.window_size - 1) / 2)
                and len(gps_window.gps_points) == gps_window.window_size
            ):

                # If the minimum is the center point of the gps_window we need to go
                # back to that location, Min index = middle

                util.Search.simple_goto_wait(
                    gps_window.gps_points[int((gps_window.window_size - 1) / 2)]
                )

                if gps_window.distance[2] <= util.LAND_THRESHOLD:
                    log.info("-- Landing")
                    AviDrone.mode = VehicleMode("LAND")
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

                util.Search.condition_yaw(180, True)
                util.Search.simple_goto_wait(
                    gps_window.gps_points[gps_window.window_size - 1]
                )
                gps_window.purge_gps_window()

            elif gps_window.get_minimum_index() == 0:
                # If the minimum data point is in the first index,
                log.info("continue forward")
                util.Search.go_to_location(util.MAGNITUDE, AviDrone.attitude.yaw, AviDrone)

            else:
                log.info(f"Did not find signal at altitude: {util.ALTITUDE}")
                log.info("Climbing...")
                util.Search.go_to_location(util.MAGNITUDE, AviDrone.attitude.yaw, AviDrone)
        time.sleep(2)


if __name__ == "__main__":
    run()
