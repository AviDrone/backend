#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    SEARCH
"""
import datetime
import logging
import os
import time

import numpy as np
from shortuuid import uuid
from dronekit import Command, VehicleMode
from parameters import (
    ALTITUDE,
    DEGREES,
    IS_TEST,
    LAND_THRESHOLD,
    MAGNITUDE,
    MISSION_TIMEOUT,
)
from pymavlink import mavutil
from transceiver.transceiver import TRANSCEIVER
from uav import AVIDRONE
from util import GPS_DATA, MISSION, NAVIGATION, VECTOR

# Logging
log = logging.getLogger(__name__)
log.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
file_handler = logging.FileHandler(os.path.join("log", "search.log"))
file_handler.setFormatter(formatter)
log.addHandler(file_handler)

class Search:
    def __init__(self):
        """
        Search
        """
        self.commands = AVIDRONE.commands
        self.width = 50
        self.length = 104
        self.height = 0
        self.phase = "primary"  # Initial phase
        self._id = str(uuid())
        self.is_enabled_primary = False
        self.is_enabled_secondary = False

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

    def get_global_pos(self):
        return self.global_frame


SEARCH = Search()

class File(Search):
    def __init__(self):
        self.dir_path = "app/missions/"
        self.file_type = "txt"
        self.file_handle = ""
        
        
    def create_filename(self, lat, lon):
        # Getting the current date and time
        date_time = datetime.now()
        self.file_handle = (f"{str(lat)}_{str(lon)}_{str(date_time)}")
        return (f"{self.dir_path}{SEARCH.phase}_{self.file_handle}_{SEARCH._id}.{self.file_type}")

    def minimize_filename(self):
        search_phase = "P" if SEARCH.phase == "primary" else "S"
        return (f"{self.dir_path}{search_phase}_{self.file_handle}{self.file_type}")

if SEARCH.phase == "primary":
    SEARCH.is_enabled_primary = True
    log.debug(f"ENABLE_PRIMARY_SEARCH: {SEARCH.is_enabled_primary}")

elif SEARCH.phase == "secondary":
    SEARCH.is_enabled_secondary = True
    log.debug(f"ENABLE_SECONDARY_SEARCH{SEARCH.is_enabled_primary}")

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

            if self.next_waypoint == VECTOR.get_range(length, search_strip_width):
                PRIMARY.reached_end = True
                self.stopping_point = self.next_waypoint
                break
            time.sleep(1)
        return self.reached_end, self.stopping_point

    def rectangular(self, width, length, height=0):
        # Initialize values
        v_dist = 0
        h_dist = 0
        step = 0
        commands = []

        max_range = PRIMARY.strip_width
        v_num = (length / PRIMARY.strip_width) - 1
        d_alt = height / v_num
        log.debug(f"d_alt: {d_alt}")
        curr_altitude = height + ALTITUDE

        _commands = AVIDRONE.commands

        log.info(" Clear any existing commands")
        _commands.clear()

        log.info(" Define/add new commands.")

        # Add MAV_CMD_NAV_TAKEOFF command.
        # This is ignored if the vehicle is already in the air.

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
            point = NAVIGATION.get_location_meters_with_alt(
                AVIDRONE.location, i[1], i[0], i[2]
            )
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
            MISSION.filename = (
                SEARCH.dir_path + SEARCH.file_name + SEARCH.ID + SEARCH.file_type
            )
            print(f"Saving to file: {MISSION.filename }")
            
            MISSION.save_mission(MISSION.filename)
        AVIDRONE.commands.clear()
        print("Mission saved")


PRIMARY = Primary()


class Secondary(Search):
    def __init__(self):
        self.is_enabled = False
        self.SIGNAL_FOUND = False
        self.timeout_counter = 0

    def search(self):
        if IS_TEST:
            beacon = TRANSCEIVER.mock_transceiver(
                TRANSCEIVER.position,
                [AVIDRONE.location],
            )

        else:
            beacon = TRANSCEIVER.read_transceiver()

        # Confirm we are in GUIDED mode
        while AVIDRONE.mode != "GUIDED":
            AVIDRONE.mode = VehicleMode("GUIDED")
            log.info("Waiting for GUIDED mode...")
            time.sleep(1)

        while AVIDRONE.mode == "GUIDED":
            if MISSION_TIMEOUT:  # return to landing
                log.critical("Reached timeout. Returning to launch site.")
                SECONDARY.is_enabled = False
                AVIDRONE.mode = VehicleMode("RTL")

            if TRANSCEIVER.get_direction(beacon[0]) < 2.0:  # Turn left
                MISSION.condition_yaw(-DEGREES, True)

            elif TRANSCEIVER.get_direction(beacon[0]) > 2.0:  # Turn right
                MISSION.condition_yaw(DEGREES, True)

            elif TRANSCEIVER.get_direction(beacon[0]) == 2.0:  # Keep straight
                GPS_DATA.add_point(SEARCH.get_global_pos(), beacon.distance)

            # If the minimum is the center point of the gps_window,
            # we need to go back to that location
            if (
                GPS_DATA.get_minimum_index() == ((GPS_DATA.window_size - 1) / 2)
                and len(GPS_DATA.gps_points) == GPS_DATA.window_size
            ):
                NAVIGATION.simple_goto_wait(
                    GPS_DATA.gps_points[int((GPS_DATA.window_size - 1) / 2)]
                )

                if GPS_DATA.distance[2] <= LAND_THRESHOLD:
                    SIGNAL_FOUND = True

                if SIGNAL_FOUND:
                    AVIDRONE.mode = VehicleMode("LAND")
                    log.warning("-- Landing")
                    current_time = datetime.datetime.now()
                    beacon.signal_found_msg()
                    log.info(
                        f"\n-------- VICTIM FOUND: {TRANSCEIVER.signal_detected} -------- "
                    )
                    log.info(f"-- Time: {current_time}")
                    log.info(f"-- Location: {AVIDRONE.location.global_frame}\n")

                else:
                    GPS_DATA.purge_gps_window()

            elif (
                GPS_DATA.get_minimum_index() == (GPS_DATA.window_size - 1)
                and len(GPS_DATA.gps_points) == GPS_DATA.window_size
            ):
                # If the minimum data point is the last one in the array we have gone
                # too far and in the wrong direction
                NAVIGATION.condition_yaw(180, True)
                NAVIGATION.simple_goto_wait(
                    GPS_DATA.gps_points[GPS_DATA.window_size - 1]
                )
                GPS_DATA.purge_gps_window()

            elif GPS_DATA.get_minimum_index() == 0:
                # If the minimum data point is in the first index,
                log.info("continue forward")
                NAVIGATION.better_goto(MAGNITUDE, AVIDRONE.attitude.yaw)

            else:
                self.timeout_counter += 1
                NAVIGATION.better_goto(MAGNITUDE, AVIDRONE.attitude.yaw)

            if self.timeout_counter == 100:
                break
            time.sleep(2)


SECONDARY = Secondary()
