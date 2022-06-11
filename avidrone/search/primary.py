#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    PRIMARY SEARCH
"""
from __future__ import print_function

import logging
import os
import time

import drone
import numpy as np
from dronekit import LocationGlobalRelative, VehicleMode
from pymavlink import mavutil

import primary_functions as pf
from search import Search
from uav import AVIDRONE
from util import get_range, save_mission

# logging
log = logging.getLogger(__name__)
log.setLevel(logging.INFO)
msg = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
file_handler = logging.FileHandler(os.path.join("log", "primary.log"))
file_handler.setFormatter(msg)
log.addHandler(file_handler)

aviDrone = drone.vehicle
sitl = drone.sitl
vector = drone.vector
mission = drone.mission
search = Search()


# Switch to secondary search mode
EN_SECONDARY_SWITCH = False

# width of the search
width = search.width

# length of the search
totalLength = search.length

# height
totalAlt = search.height

# search strip size
dLength = 6  # meters


# set RTL for battery low
pf.set_FS_BATT()

pf.battery_information()

# Save search to file.
SAVE_FILE = False
if SAVE_FILE:
    mission_file = "r_mission3.txt"
    print("Saving to file", mission_file)
    pf.save_search_to_file(mission_file, totalAlt, width, dLength, totalLength)

# Start in Guided mode
print("Set mode to GUIDED: ")
aviDrone.mode = VehicleMode("GUIDED")

# Set mission for drone angle
print("Create a new mission (for current location)")
my_angle = 360 - np.degrees(aviDrone.attitude.yaw)
print("Drone angle: %s" % my_angle)

# Here, the drone goes into a flat plane search or altitude search depending on altitude
if totalAlt == 0:
    pf.rectangular_primary_search_basic(
        aviDrone.location.global_frame, width, dLength, totalLength, my_angle
    )
else:
    pf.rectangular_primary_search_with_alt(
        aviDrone.location.global_frame, width, dLength, totalLength, totalAlt, my_angle
    )

# take off, going to desired altitude (GUIDED mode)
mission.arm_and_takeoff(
    20 + totalAlt
)  # Note: At low (<10) altitudes, vehicle may crash.

# Start mission
# ------------------------------------------------------------
print("Starting mission")
aviDrone.commands.next = 0

# Set mode to AUTO to start mission
print("Set vehicle mode to AUTO")
aviDrone.mode = VehicleMode("AUTO")
print("final waypoint: %s" % (get_range(totalLength, dLength)))

# stay in follow_primary function during search
reached_end, stopping_point = pf.follow_primary(totalLength, dLength)
if not reached_end:
    print("holding alt")
    aviDrone.mode = VehicleMode("GUIDED")
    hold_count = 0
    hold_sec = 3  # amount of time we hold

    while True:
        if aviDrone.mode != "GUIDED":
            print("Not holding alt")
            time.sleep(1)

        # hold altitude
        if aviDrone.location.global_frame.alt < 10:
            a_location = LocationGlobalRelative(
                aviDrone.location.global_frame.lat,
                aviDrone.location.global_frame.lon,
                10,
            )
            aviDrone.simple_goto(a_location)
        if hold_count != 0:
            time.sleep(hold_sec)
            break
        hold_count += 1

    aviDrone.mode = VehicleMode("GUIDED")
    time.sleep(1)
    pf.reupload_commands(stopping_point)
    print("Reuploaded size", aviDrone.commands.count)
    time.sleep(1)

# Restart search from here.
# Future work could be to combine this portion into the portion above for less duplicate code.
reached_end_again = False
if not reached_end:
    while not reached_end_again:
        pf.reupload_commands(stopping_point)
        print("Reuploaded size", aviDrone.commands.count)
        print("Set vehicle mode to AUTO")
        aviDrone.mode = VehicleMode("AUTO")
        time.sleep(1)

        reached_end_again, stopping_point2 = pf.follow_primary(totalLength, dLength)

        print("holding alt again")
        aviDrone.mode = VehicleMode("GUIDED")
        while True:
            if aviDrone.mode != "GUIDED":
                print("Not holding alt")
                time.sleep(1)

            # hold altitude
            if aviDrone.location.global_frame.alt < 10:
                a_location = LocationGlobalRelative(
                    aviDrone.location.global_frame.lat,
                    aviDrone.location.global_frame.lon,
                    10,
                )
                aviDrone.simple_goto(a_location)
            if hold_count != 0:
                time.sleep(hold_sec)
                break
            hold_count += 1

# Return to launch if search complete
if reached_end or reached_end_again:
    print("Return to launch")
    aviDrone.mode = VehicleMode("RTL")
    EN_SECONDARY_SWITCH = True
    save_mission("mission.txt")


print("Switch to secondary:")
if EN_SECONDARY_SWITCH and not reached_end:
    # import secondary
    aviDrone.mode = VehicleMode("ALT_HOLD")
    print("-- Switch to secondary successful -- ")
    # secondary.run()


pf.battery_information()
# Close vehicle object before exiting script
print("Close vehicle object")
aviDrone.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()
