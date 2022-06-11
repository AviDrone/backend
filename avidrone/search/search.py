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
from pymavlink import mavutil

from params import ALTITUDE
from transceiver.transceiver import TRANSCEIVER
from uav import AVIDRONE
from util import MISSION, Vector

# Logging
log = logging.getLogger(__name__)
log.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
file_handler = logging.FileHandler(os.path.join("log", "search.log"))
file_handler.setFormatter(formatter)
log.addHandler(file_handler)

log.info("**************** SEARCH ****************")


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
        self.file_name = "Primary-"
        self.file_type = ".txt"
        self.dir_path = "missions/"

    #    TODO add utm/lat-long conversion functions here
    def get_range(self, totalLength, dLength):
        return (totalLength / dLength) * 2

    # Any condition we want to break the primary search can be done in this command.
    # This will be called repeatedly and return true when the break condition is true.
    def break_condition(self):
        next_waypoint = self.avidrone.commands.next
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

    def rectangular(self, place_h1, place_h2, location, width, length, angle, height=0):
        max_range = TRANSCEIVER.curr_search_strip_width
        _commands = AVIDRONE.commands
        commands = []
        arr = []

        v_dist = h_dist = step = 0

        _commands.clear()  # Clears any existing commands
        _commands.add(self.start_waypoint)

        for _ in range(1, max_range):
            if step == 0:
                h_dist = 0
            elif (step == 1) or (step == 3):
                v_dist += length
            else:
                h_dist = width
            step += 1
            step = step % 4
            arr.append([h_dist, v_dist, 0])

        # Rotation
        initial_vector = np.asarray((arr[1][0], arr[1][1], 0))
        final_vector = Vector.rotate_vector(self, initial_vector, angle)

        if np.array_equal(final_vector, initial_vector):
            # do not rotate if equal
            rotated_vector = np.array(arr)
        else:
            # otherwise, rotate
            rotated_vector = Vector.rotate_cloud(arr, initial_vector, final_vector)

        # def get_location_meters_with_alt(self, original_location, dNorth, dEast, newAlt):
        for points in rotated_vector:
            point = MISSION.get_location_meters_with_alt(
                AVIDRONE.location, points[1], points[0], points[2]
            )
            wp_command = Command(
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
                ALTITUDE,
            )
            _commands.add(wp_command)
            commands.append(_commands)

        if height == 0:
            # add dummy waypoint at final point (lets us know when have reached destination)
            final_wp = Command(
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

        # add dummy waypoint at final point (lets us know when have reached destination)
        else:
            final_wp = Command(
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
        _commands.add(final_wp)
        commands.append(_commands)
        log.info(" Upload new commands to vehicle")
        _commands.upload()

    def save_to_file(self, text_file, width, length, height):
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
        angle = 360 - np.degrees(AVIDRONE.yaw)

        PRIMARY.rectangular(AVIDRONE.location, width, length, angle, angle, height)

        print("returning to launch")
        self.return_to_launch()
        if SEARCH.SAVE:
            mission_file = SEARCH.file_name + SEARCH.ID + SEARCH.file_type
            print(f"Saving to file: {mission_file}")
        AVIDRONE.commands.clear()
        print("Mission saved")


PRIMARY = Primary()


class Secondary(Search):
    def __init__(self):
        self.is_enabled = False


SECONDARY = Secondary()
