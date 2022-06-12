#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    SEARCH
"""
import logging
import os
import time
from operator import le

import numpy as np
import shortuuid
from dronekit import Command, VehicleMode
from params import ALTITUDE
from pymavlink import mavutil
from transceiver.transceiver import TRANSCEIVER
from uav import AVIDRONE
from util import MISSION, VECTOR

# Logging
log = logging.getLogger(__name__)
log.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
file_handler = logging.FileHandler(os.path.join("log", "search.log"))
file_handler.setFormatter(formatter)
log.addHandler(file_handler)


class Search:
    def __init__(self):
        self.commands = AVIDRONE.commands
        self.width = 50
        self.length = 100
        self.height = 0
        self.phase = "primary"  # TODO implement state machine here
        self.ENABLE_PRIMARY_SEARCH = False
        self.ENABLE_SECONDARY_SEARCH = False

        # Mission file settings
        self.SAVE = True
        self.ID = str(shortuuid.uuid())
        self.file_name = "primary-"
        self.file_type = ".txt"
        self.dir_path = "missions/"

    # TODO move this to util.vector
    #    TODO add utm/lat-long conversion functions here
    def get_range(self, totalLength, dLength):
        return (totalLength / dLength) * 2

    # Any condition we want to break the primary search can be done in this command.
    # This will be called repeatedly and return true when the break condition is true.
    def break_condition(self):
        next_waypoint = AVIDRONE.commands.next
        if next_waypoint == 40:
            AVIDRONE.mode = VehicleMode("GUIDED")
            print("breaking...")
            return True
        return False

    def return_to_launch(self):
        AVIDRONE.commands.add(
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


SEARCH = Search()


if SEARCH.phase == "primary":
    SEARCH.ENABLE_PRIMARY_SEARCH = True
    log.debug(f"ENABLE_PRIMARY_SEARCH: {SEARCH.ENABLE_PRIMARY_SEARCH}")

elif SEARCH.phase == "secondary":
    SEARCH.ENABLE_SECONDARY_SEARCH = True
    log.debug(f"ENABLE_SECONDARY_SEARCH{SEARCH.ENABLE_PRIMARY_SEARCH}")

else:
    log.error("Unknown mode")


class Primary(Search):
    def __init__(self):
        self.strip_width = TRANSCEIVER.curr_search_strip_width
        self.is_enabled = False
        self.reached_end = False
        self.stopping_point = None
        self.next_waypoint = None
        self.start_waypoint = Command(
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

    def run(self, length, search_strip_width):
        self.is_enabled = True

        if AVIDRONE.mode != "AUTO":
            time.sleep(1)

        while self.is_enabled:
            self.next_waypoint = AVIDRONE.commands.next

            if SEARCH.break_condition:
                time.sleep(1)
                AVIDRONE.commands.clear()
                AVIDRONE.commands.upload()
                time.sleep(1)
                self.stopping_point = self.next_waypoint

            if self.next_waypoint == self.get_range(length, search_strip_width):
                PRIMARY.reached_end = True
                self.stopping_point = self.next_waypoint
                break
            time.sleep(1)
        return self.reached_end, self.stopping_point

    def rectangular(self, width, length, height=0):
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
        max_range = PRIMARY.strip_width
        v_num = (length/ PRIMARY.strip_width) - 1
        d_alt = height / v_num
        log.debug(f"d_alt: {d_alt}")
        curr_altitude = height + ALTITUDE

        _commands = AVIDRONE.commands

        print(" Clear any existing commands")
        _commands.clear()

        print(" Define/add new commands.")
        # Add new commands. The meaning/order of the parameters is documented in the Command class.

        # Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
        _commands.add(
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
        commands = []

        # Generate points above origin
        for i in range(1, int(max_range)):
            if step == 0:
                h_dist = 0
                if i > 1:
                    curr_altitude -= d_alt
            elif (step == 1) or (step == 3):
                v_dist += PRIMARY.strip_width
            else:
                h_dist = width
                curr_altitude -= d_alt
            step += 1
            step = step % 4

            # add points to array
            commands.append([h_dist, v_dist, curr_altitude])

        # setup rotation
        commands = np.asarray(commands)
        vector1 = (commands[1][0], commands[1][1], 0)
        vector1 = np.asarray(vector1)
        vector2 = VECTOR.rotate_vector(vector1, AVIDRONE.angle)

        # avoid rare case where a divide by 0 occurs if vector1 = vector2
        if np.array_equal(vector2, vector1):
            # do not rotate if equal
            rotated = commands
        else:
            # otherwise, rotate
            rotated = VECTOR.rotate_cloud(commands, vector1, vector2)

        # rotate points
        for i in rotated:
            point = MISSION.get_location_meters_with_alt(AVIDRONE.location, i[1], i[0], i[2])
            _commands.add(
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
        _commands.add(
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
        _commands.upload()

    def save_to_file(self, width, length, height):
        print("adding takeoff to altitude ", ALTITUDE)
        AVIDRONE.commands.add(
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
        print("adding mission")
        
        PRIMARY.rectangular(width, length, height)

        print("returning to launch")
        self.return_to_launch()
        if SEARCH.SAVE:
            mission_file = SEARCH.dir_path + SEARCH.file_name + SEARCH.ID + SEARCH.file_type
            print(f"Saving to file: {mission_file}")
            MISSION.save_mission(mission_file)
        AVIDRONE.commands.clear()
        print("Mission saved")

PRIMARY = Primary()


class Secondary(Search):
    def __init__(self):
        self.is_enabled = False


SECONDARY = Secondary()
