#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    PRIMARY FUNCTIONS
"""
from __future__ import print_function

# Set up option parsing to get connection string
import argparse
import logging as log
import time

import drone
import numpy as np
from dronekit import Command, LocationGlobalRelative, VehicleMode
from pymavlink import mavutil
from util import (
    ALTITUDE,
    get_distance_metres,
    get_location_metres,
    get_location_metres_with_alt,
    get_range,
)

aviDrone = drone.vehicle
sitl = drone.sitl
vector = drone.vector
mission = drone.mission

# set to RTL mode if battery low (https://github.com/ArduPilot/ardupilot_wiki/issues/291)
def set_FS_BATT():
    aviDrone.parameters.set("FS_BATT_ENABLE", 2)


def print_parameters():
    print("\nPrint all parameters (iterate `aviDrone.parameters`):")
    for key, value in aviDrone.parameters.items():
        print(" Key:%s Value:%s" % (key, value))


def battery_information():
    print("Level:", aviDrone.battery.level)
    print("Voltage:", aviDrone.battery.voltage)
    print("Current:", aviDrone.battery.current)
    print("Battery:", aviDrone.battery)


# Rectangular search taking angle and altitude into account
def rectangular_primary_search_with_alt(
    a_location, width, dLength, totalLength, totalAlt, angle
):
    """
    Primary search over a sloped plane in the direction of a specified angle,
        based on dronekit's basic square mission:
        https://dronekit-python.readthedocs.io/en/latest/examples/mission_basic.html#example-mission-basic

    a_location: LocationGlobal data type, expected to be drone's current location
    width: width in meters (horizontal stretch)
    dLength: search strip size in meters (small vertical stretches)
    totalLength: length in meters (vertical stretch)
    totalAlt: height of slop in meters (height of 'mountain')
    angle: in degrees

    """

    print("Running rectangular_primary_search_with_alt")

    # calculated values
    maxrange = get_range(totalLength, dLength)
    v_num = (totalLength / dLength) - 1
    dAlt = totalAlt / v_num
    print("dAlt: %s" % dAlt)
    myAlt = totalAlt + 10

    cmds = aviDrone.commands

    print(" Clear any existing commands")
    cmds.clear()

    print(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class.

    # Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(
        Command(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            10,
        )
    )

    # Initialize values
    v_dist = 0
    h_dist = 0
    step = 0
    initArray = []

    # Generate points above origin
    for i in range(1, int(maxrange)):
        if step == 0:
            h_dist = 0
            if i > 1:
                myAlt -= dAlt
        elif (step == 1) or (step == 3):
            v_dist += dLength
        else:
            h_dist = width
            myAlt -= dAlt
        step += 1
        step = step % 4

        # add points to array
        initArray.append([h_dist, v_dist, myAlt])

    # setup rotation
    initArray = np.asarray(initArray)
    vector1 = (initArray[1][0], initArray[1][1], 0)
    vector1 = np.asarray(vector1)
    vector2 = vector.rotate_vector(vector1, angle)

    # avoid rare case where a divide by 0 occurs if vector1 = vector2
    if np.array_equal(vector2, vector1):
        # do not rotate if equal
        rotated = initArray
    else:
        # otherwise, rotate
        rotated = vector.rotate_cloud(initArray, vector1, vector2)

    # rotate points
    for i in rotated:
        point = get_location_metres_with_alt(a_location, i[1], i[0], i[2])
        cmds.add(
            Command(
                0,
                0,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                0,
                0,
                0,
                0,
                0,
                point.lat,
                point.lon,
                point.alt,
            )
        )

    # add dummy waypoint at final point (lets us know when have reached destination)
    cmds.add(
        Command(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,
            0,
            0,
            0,
            0,
            0,
            point.lat,
            point.lon,
            0,
        )
    )

    print(" Upload new commands to vehicle")
    cmds.upload()


def reupload_commands(index):
    cmds = aviDrone.commands
    for command in aviDrone_commands_copy:
        cmds.add(command)
    print("nxt1:", cmds.next)
    cmds.upload()
    time.sleep(1)
    # go back one when we check again. Further testing required to see if this. is helpful
    cmds.next = index - 1
    print("nxt2:", cmds.next)


aviDrone_commands_copy = []
# Rectangular search over a flat plane taking angle into account
def rectangular_primary_search_basic(a_location, width, dLength, totalLength, angle):
    """
    Primary search over a flat plane in the direction of a specified angle,
        based on dronekit's basic square mission:
        https://dronekit-python.readthedocs.io/en/latest/examples/mission_basic.html#example-mission-basic

    a_location: LocationGlobal data type, expected to be drone's current location
    width: width in meters (horizontal stretch)
    dLength: search strip size in meters (small vertical stretches)
    totalLength: length in meters (vertical stretch)
    angle: in degrees

    """

    print("Running rectangular_primary_search_basic")

    cmds = aviDrone.commands

    print(" Clear any existing commands")
    cmds.clear()

    print(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class.

    # Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    start = Command(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        10,
    )
    cmds.add(start)
    maxrange = get_range(totalLength, dLength)
    v_dist = 0
    h_dist = 0
    step = 0
    initArray = []
    for i in range(1, int(maxrange)):
        if step == 0:
            h_dist = 0
        elif (step == 1) or (step == 3):
            v_dist += dLength
        else:
            h_dist = width
        step += 1
        step = step % 4
        initArray.append([h_dist, v_dist, 0])

    # setup rotation
    initArray = np.asarray(initArray)
    vector1 = (initArray[1][0], initArray[1][1], 0)
    vector1 = np.asarray(vector1)
    vector2 = vector.rotate_vector(vector1, angle)
    rotated = vector.rotate_cloud(initArray, vector1, vector2)

    # rotate points in array
    print("Rotating")
    for i in rotated:
        point = get_location_metres(a_location, i[1], i[0])
        myCommand = Command(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,
            0,
            0,
            0,
            0,
            0,
            point.lat,
            point.lon,
            0,
        )
        cmds.add(myCommand)
        aviDrone_commands_copy.append(myCommand)

    # add dummy waypoint at final point (lets us know when have reached destination)
    dummy = Command(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,
        0,
        0,
        0,
        0,
        0,
        point.lat,
        point.lon,
        0,
    )
    cmds.add(dummy)
    aviDrone_commands_copy.append(dummy)
    print(" Upload new commands to vehicle")
    cmds.upload()


def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint.
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = aviDrone.commands.next
    if nextwaypoint == 0:
        return None
    missionitem = aviDrone.commands[nextwaypoint - 1]  # commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat, lon, alt)
    distancetopoint = get_distance_metres(
        aviDrone.location.global_frame, targetWaypointLocation
    )
    return distancetopoint


def download_mission():
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    missionlist = []
    cmds = aviDrone.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist


def save_mission(aFileName):
    """
    Save a mission in the Waypoint file format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    missionlist = download_mission()
    output = "QGC WPL 110\n"
    for cmd in missionlist:
        commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
            cmd.seq,
            cmd.current,
            cmd.frame,
            cmd.command,
            cmd.param1,
            cmd.param2,
            cmd.param3,
            cmd.param4,
            cmd.x,
            cmd.y,
            cmd.z,
            cmd.autocontinue,
        )
        output += commandline
    with open(aFileName, "w") as file_:
        file_.write(output)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don not let the user try to arm until autopilot is ready
    while not aviDrone.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    aviDrone.mode = VehicleMode("GUIDED")
    aviDrone.armed = True

    while not aviDrone.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    aviDrone.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", aviDrone.location.global_relative_frame.alt)
        if (
            aviDrone.location.global_relative_frame.alt >= aTargetAltitude * 0.95
        ):  # Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)


def return_to_launch():
    aviDrone.commands.add(
        Command(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
    )


def save_search_to_file(file, totalAlt, width, dLength, totalLength):
    # aviDrone.simple_takeoff(ALTITUDE)
    print("adding takeoff to altitude ", ALTITUDE)
    aviDrone.commands.add(
        Command(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            ALTITUDE,
        )
    )
    print("adding mission")
    my_angle = 360 - np.degrees(aviDrone.attitude.yaw)
    if totalAlt == 0:
        rectangular_primary_search_basic(
            aviDrone.location.global_frame, width, dLength, totalLength, my_angle
        )
    else:
        rectangular_primary_search_with_alt(
            aviDrone.location.global_frame,
            width,
            dLength,
            totalLength,
            totalAlt,
            my_angle,
        )
    print("returning to launch")
    # return_to_launch()
    save_mission(file)
    aviDrone.commands.clear()
    print("Mission saved")


def follow_primary(totalLength, dLength):
    reached_end = False
    stopping_point = 0
    while True:
        battery_information()
        if aviDrone.mode != "AUTO":
            time.sleep(1)

        nextwaypoint = aviDrone.commands.next
        print(
            "Distance to waypoint (%s): %s"
            % (nextwaypoint, distance_to_current_waypoint())
        )

        if mission.break_condition():
            # print("Size of commands break", len(aviDrone.commands))
            time.sleep(1)
            aviDrone.commands.clear()
            aviDrone.commands.upload()
            time.sleep(1)
            stopping_point = nextwaypoint
            break

        if nextwaypoint == get_range(
            totalLength, dLength
        ):  # Dummy waypoint - as soon as we reach last waypoint this is true and we exit.
            print(
                "Exit 'standard' mission when start heading to final waypoint (%s)"
                % nextwaypoint
            )
            reached_end = True
            stopping_point = nextwaypoint
            break
        time.sleep(1)
    return reached_end, stopping_point
