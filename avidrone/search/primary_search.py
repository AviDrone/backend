#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    PRIMARY SEARCH
"""
from __future__ import print_function

# Set up option parsing to get connection string
# Set up option parsing to get connection string
import argparse
import asyncio
import datetime
import dronekit_sitl
import logging as log
import math
import time

from dronekit import (
    Command,
    LocationGlobal,
    LocationGlobalRelative,
    VehicleMode,
    connect,
)
from pymavlink import mavutil

parser = argparse.ArgumentParser(description="Demonstrates basic mission operations.")
parser.add_argument(
    "--connect",
    help="vehicle connection target string. If not specified, SITL automatically started and used.",
)
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Connect to the vehicle
log.info("Connecting to vehicle on: %s", connection_string)
vehicle = connect(connection_string, wait_ready=True)

# Start SITL if no connection string specified
if not connection_string:
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobal(newlat, newlon, original_location.alt)


def rectangular_primary_search(a_location, length, width):
    """
    Primary search testing, based on dronekit's basic square mission
    """

    vehicle_commands = vehicle.commands

    print(" Clear any existing commands")
    vehicle_commands.clear()

    print(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class.

    # Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    vehicle_commands.add(
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

    v_dist = 0
    h_dist = 0
    step = 0
    for i in range(1, 9):
        print("Waypoint %s and step = %s" % (i, step))
        if step == 0:
            h_dist = 0
        elif (step == 1) or (step == 3):
            v_dist -= width
        else:
            h_dist = -length
        step += 1
        step = step % 4
        point = get_location_metres(a_location, v_dist, h_dist)
        vehicle_commands.add(
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
                11,
            )
        )

    # add dummy waypoint at final point (lets us know when have reached destination)
    vehicle_commands.add(
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
            16,
        )
    )

    print(" Upload new commands to vehicle")
    vehicle_commands.upload()


def get_distance_metres(a_location1, a_location2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    d_lat = a_location2.lat - a_location1.lat
    d_long = a_location2.lon - a_location1.lon
    return math.sqrt((d_lat * d_lat) + (d_long * d_long)) * 1.113195e5


def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint.
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint == 0:
        return None
    missionitem = vehicle.commands[nextwaypoint - 1]  # commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat, lon, alt)
    distancetopoint = get_distance_metres(
        vehicle.location.global_frame, targetWaypointLocation
    )
    return distancetopoint


def download_mission():
    """
    Download the current mission from the vehicle.
    """
    vehicle_commands = vehicle.commands
    vehicle_commands.download()
    vehicle_commands.wait_ready()  # wait until download is complete.


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if (
            vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95
        ):  # Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)


print("Create a new mission (for current location)")
my_length = 200
my_width = 50
rectangular_primary_search(vehicle.location.global_frame, my_length, my_width)

# From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
arm_and_takeoff(10)

print("Starting mission")
# Reset mission set to first (0) waypoint
vehicle.commands.next = 0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")


# Monitor mission.
# Demonstrates getting and setting the command number
# Uses distance_to_current_waypoint(), a convenience function for finding the
#   distance to the next waypoint.

while True:
    nextwaypoint = vehicle.commands.next
    print(
        "Distance to waypoint (%s): %s" % (nextwaypoint, distance_to_current_waypoint())
    )

    # if nextwaypoint==3: #Skip to next waypoint
    #     print('Skipping to Waypoint 5 when reach waypoint 3')
    #     vehicle.commands.next = 5
    if (
        nextwaypoint == 15
    ):  # Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
        print(
            "Exit 'standard' mission when start heading to final waypoint (%s)"
            % nextwaypoint
        )
        break
    time.sleep(1)

print("Return to launch")
vehicle.mode = VehicleMode("RTL")


# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()
