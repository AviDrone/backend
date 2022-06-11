#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    SEARCH UTIL
"""
from __future__ import print_function

import argparse
import asyncio
import datetime
import logging
import math
import os
import time

import numpy as np

from dronekit import (
    Command,
    LocationGlobal,
    LocationGlobalRelative,
    VehicleMode,
)

from pymavlink import mavutil

from params import (
    ALTITUDE,
    DEGREE_ERROR,
    DEGREES,
    DISTANCE_ERROR,
    IS_TEST,
    LAND_THRESHOLD,
    MAGNITUDE,
    WINDOW_SIZE,
    WITH_TRANSCEIVER,
)

# logging
log = logging.getLogger(__name__)
log.setLevel(logging.INFO)
msg = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
file_handler = logging.FileHandler(os.path.join("log", "util.log"))
file_handler.setFormatter(msg)
log.addHandler(file_handler)

from uav import AVIDRONE
if WITH_TRANSCEIVER:
    from transceiver.transceiver import TRANSCEIVER
    log.debug(f"transceiver model: {TRANSCEIVER.curr_model}")
    log.debug(f"search strip width: {TRANSCEIVER.curr_search_strip_width}")


class GpsData:
    def __init__(self):
        self.window_size = WINDOW_SIZE
        self.gps_points = []
        self.distance = []

    def add_point(self, new_gps_point, new_distance):
        self.gps_points.insert(0, new_gps_point)
        del self.gps_points[self.window_size :]
        self.distance.insert(0, new_distance)
        del self.distance[self.window_size :]

    def get_minimum_index(self):
        min_index = 0
        for i in range(0, len(self.distance)):
            if self.distance[i] < self.distance[min_index]:
                min_index = i
        return min_index

    def purge_gps_window(self):
        self.gps_points.clear()
        self.distance.clear()

GPS_DATA = GpsData()    # singleton

class Mission:
    def __init__(self):
        # TODO Remove this comment: replace with UAV singleton
        self.mavutil = mavutil
        self.global_frame = AVIDRONE.altitude
        self.original_yaw = AVIDRONE.yaw
        self.heading = -1  # TODO get correct value
        self.angle = 360 - AVIDRONE.yaw
        self.relative = False
        self.cw = -1  # TODO get correct value


    def arm_and_takeoff(self, target_altitude):
        # Pre-arm check: Prevent user try to arm until autopilot is ready
        log.info(" Waiting for vehicle to initialize...")
        while not AVIDRONE.is_armable:
            log.debug("Arming motors")  # Copter should arm in GUIDED mode
            time.sleep(1)
            AVIDRONE.mode = VehicleMode("GUIDED")
            AVIDRONE.armed = True

            while not AVIDRONE.armed:
                log.info(" Waiting for arming...")
                time.sleep(1)

        log.warning("Taking off!")
        AVIDRONE.simple_takeoff(target_altitude)  # Take off to target altitude

        # Wait until the vehicle reaches a safe height before continuing

        # otherwise the command after Vehicle.simple_takeoff will execute immediately.
        
        while True:
            print(" Altitude: ", AVIDRONE.altitude)
            time.sleep(1)
            if (
                AVIDRONE.altitude
                >= target_altitude * 0.95
            ):  # Trigger just below target alt.
                print("Reached target altitude")
                break

    def download_mission(self):
        """
        Downloads the current mission and returns it in a list.
        It is used in save_mission() to get the file information to save.
        """
        missions = []
        commands = AVIDRONE.commands
        commands.download()
        commands.wait_ready()
        for command in commands:
            missions.append(command)
        return missions

    def save_mission(self, text_file):
        """
        Save a mission in the Waypoint file format:
        (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file
        """
        missions = self.download_mission()
        output = "QGC WPL 110\n"
        for cmd in missions:
            command_line = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
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
            output += command_line
        with open(text_file, "w") as file_:
            file_.write(output)



MISSION =  Mission()

class Vector:
    def __init__(self):
        vector_1 = []
        vector_2 = []
        self.vector_1 = np.asarray(vector_1)
        self.vector_2 = np.asarray(vector_2)

    @staticmethod
    def rotate_cloud(Points, V1, V2):
        # V1 is the current vector which the coordinate system is aligned to
        # V2 is the vector we want the system aligned to
        # Points is an (n,3) array of n points (x,y,z)
        V1 = np.asarray(V1)
        V2 = np.asarray(V2)

        # Normalize V1 and V2 in case they aren't already
        V1Len = (V1[0] ** 2 + V1[1] ** 2 + V1[2] ** 2) ** 0.5
        V2Len = (V2[0] ** 2 + V2[1] ** 2 + V2[2] ** 2) ** 0.5
        V1 = V1 / V1Len
        V2 = V2 / V2Len

        # Calculate the vector cross product
        V1V2Cross = np.cross(V1, V2)
        V1V2CrossNorm = (
            V1V2Cross[0] ** 2 + V1V2Cross[1] ** 2 + V1V2Cross[2] ** 2
        ) ** 0.5
        V1V2CrossNormalized = V1V2Cross / V1V2CrossNorm

        # Dot product
        V1V2Dot = np.dot(V1, V2)
        V1Norm = (V1[0] ** 2 + V1[1] ** 2 + V1[2] ** 2) ** 0.5
        V2Norm = (V2[0] ** 2 + V2[1] ** 2 + V2[2] ** 2) ** 0.5

        # angle between the vectors
        Theta = np.arccos(V1V2Dot / (V1Norm * V2Norm))

        # Using Rodriguez's rotation formula (wikipedia):
        e = V1V2CrossNormalized
        pts_rotated = np.empty((len(Points), 3))
        if np.size(Points) == 3:
            p = Points
            p_rotated = (
                np.cos(Theta) * p
                + np.sin(Theta) * (np.cross(e, p))
                + (1 - np.cos(Theta)) * np.dot(e, p) * e
            )
            pts_rotated = p_rotated
        else:
            for i, p in enumerate(Points):
                p_rotated = (
                    np.cos(Theta) * p
                    + np.sin(Theta) * (np.cross(e, p))
                    + (1 - np.cos(Theta)) * np.dot(e, p) * e
                )
                pts_rotated[i] = p_rotated
        return pts_rotated

    @staticmethod
    def rotate_vector(vector, angle):
        # vector is the vector being rotated
        # angle is used to rotate vector and is given in degrees

        # Convert angle to radians
        Angle_Rad = np.radians(angle)

        # rotation matrix
        # See https://en.wikipedia.org/wiki/Rotation_matrix for more information
        r = np.array(
            (
                (np.cos(Angle_Rad), -np.sin(Angle_Rad)),
                (np.sin(Angle_Rad), np.cos(Angle_Rad)),
            )
        )

        # we only care about x and y, not z
        a_vector = (vector[0], vector[1])
        a_vector = np.asarray(a_vector)

        # vector after rotation
        rotated = r.dot(a_vector)

        # return 3D vector
        NewVector = (rotated[0], rotated[1], vector[2])

        return NewVector


#     def better_get_distance_meters(self, a_location1, a_location2):
#         lat1, lat2 = a_location1.lat, a_location2.lat
#         lon1, lon2 = a_location1.lon, a_location2.lon

#         # 6371.009e3 is the mean radius of the earth that gives us the distance
#         # (NOT USED)
#         #
#         # 1.113195e5 gives us meters
#         distance = (
#             2
#             * math.asin(
#                 math.sqrt(
#                     (math.sin((lat1 - lat2) / 2)) ** 2
#                     + math.cos(lat1)
#                     * math.cos(lat2)
#                     * (math.sin((lon1 - lon2) / 2)) ** 2
#                 )
#             )
#             * 1.113195e5
#         )
#         return distance

#     def better_get_location_meters(self, original_location, distance, angle):
#         lat = math.asin(
#             math.sin(original_location.lat) * math.cos(distance)
#             + math.cos(original_location.lat) * math.sin(distance) * math.cos(angle)
#         )

#         d_lon = math.atan2(
#             math.sin(angle) * math.sin(distance) * math.cos(original_location.lat),
#             math.cos(distance) - math.sin(original_location.lat) * math.sin(lat),
#         )

#         lon = (original_location.lon - d_lon + math.pi) % (2 * math.pi) - math.pi

#         return LocationGlobal(lat, lon, original_location.alt)

#     def better_goto(self, distance, angle):
#         """
#         Moves the vehicle to a position d_north metres North and d_east metres East of the current position.
#         The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for
#         the target position. This allows it to be called with different position-setting commands.
#         By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().
#         The method reports the distance to target every two seconds.

#         Retrieved from this link:
#         https://dronekit-python.readthedocs.io/en/latest/examples/guided-set-speed-yaw-demo.html
#         """

#         currentLocation = (
#             self.avidrone.location.global_frame
#         )  # was global_relative_frame
#         targetLocation = self.better_get_location_meters(
#             currentLocation, distance, angle
#         )

#         self.avidrone.simple_goto(targetLocation)

#         loop_count = 0
#         while (
#             self.avidrone.mode.name == "GUIDED"
#         ):  # Stop action if we are no longer in guided mode.
#             # print "DEBUG: mode: %s" % vehicle.mode.name
#             remainingDistance = self.better_get_distance_meters(
#                 self.avidrone.location.global_frame, targetLocation
#             )
#             # global_frame was global_relative_frame
#             # remainingDistance=get_distance_meters(vehicle.location.global_frame, targetLocation)
#             # #global_frame was global_relative_frame
#             print("Distance to target: ", remainingDistance)
#             # if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.

#             loop_count += 1

#             if remainingDistance <= 0.35:
#                 print("Reached target")
#                 break
#             elif loop_count >= 10:
#                 print("Stuck, skipping target")
#                 break
#             time.sleep(2)

#     # Any condition we want to break the primary search can be done in this command.
#     # This will be called repeatedly and return true when the break condition is true.
#     def break_condition(self):
#         next_waypoint = self.avidrone.commands.next
#         if next_waypoint == 40:
#             self.avidrone.mode = VehicleMode("GUIDED")
#             print("breaking...")
#             return True
#         return False

#     def condition_yaw(self, heading, relative):
#         if relative:
#             is_relative = True  # yaw relative to direction of travel
#         else:
#             is_relative = False  # yaw is an absolute angle

#         if heading < 0:
#             heading = abs(heading)
#             cw = -1
#         else:
#             cw = 1

#         msg = self.avidrone.message_factory.command_long_encode(
#             0,  # target system
#             0,  # target component
#             self.mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
#             0,  # confirmation
#             heading,  # param 1, yaw in degrees
#             0,  # param 2, yaw speed deg/s
#             cw,  # param 3, direction -1 ccw, 1 cw
#             is_relative,  # param 4, relative offset 1, absolute angle 0
#             0,
#             0,
#             0,
#         )  # param 5 ~ 7 not used
#         self.avidrone.send_mavlink(msg)  # send command to vehicle

#     def forward_calculation(self):
#         flight_direction = []
#         yaw = drone.vehicle.attitude.yaw

#         print(yaw)
#         flight_direction.append(MAGNITUDE * math.cos(yaw))
#         flight_direction.append(MAGNITUDE * math.sin(yaw))

#         return flight_direction

#     def simple_goto_wait(self, go_to_checkpoint):
#         self.avidrone.simple_goto(go_to_checkpoint)
#         global_frame = self.avidrone.location.global_frame
#         distance = get_distance_metres(
#             Search.get_global_pos(global_frame), go_to_checkpoint
#         )

#         while distance >= DISTANCE_ERROR and self.avidrone.mode.name == "GUIDED":
#             print(distance)
#             distance = get_distance_metres(
#                 Search.get_global_pos(global_frame), go_to_checkpoint
#             )
#             time.sleep(1)

#         if self.avidrone.mode.name != "GUIDED":
#             self.avidrone.simple_goto(self.avidrone.location.global_frame)
#             print("Halting simple_go_to")

#         print("Checkpoint reached")

#     def takeoff_to_altitude(self):
#         self.avidrone.simple_takeoff(ALTITUDE)
#         log.info(f"-- Taking off to altitude (m): {ALTITUDE} \n")

#         while True:
#             current_alt = self.avidrone.location.global_relative_frame.alt
#             if current_alt >= ALTITUDE * 0.95:
#                 log.info(f"-- Reached {ALTITUDE}m")
#                 break
#             time.sleep(1)

#     def return_to_launch(self):
#         AVIDRONE.commands.add(
#             Command(
#                 0,
#                 0,
#                 0,
#                 mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#                 mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
#                 0,
#                 0,
#                 0,
#                 0,
#                 0,
#                 0,
#                 0,
#                 0,
#                 0,
#             )
#     )



#     def save_mission(self, text_file):
#         """
#         Save a mission in the Waypoint file format:
#         (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file
#         """
#         missions = self.download_mission()
#         output = "QGC WPL 110\n"
#         for cmd in missions:
#             command_line = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
#                 cmd.seq,
#                 cmd.current,
#                 cmd.frame,
#                 cmd.command,
#                 cmd.param1,
#                 cmd.param2,
#                 cmd.param3,
#                 cmd.param4,
#                 cmd.x,
#                 cmd.y,
#                 cmd.z,
#                 cmd.autocontinue,
#             )
#             output += command_line
#         with open(text_file, "w") as file_:
#             file_.write(output)

#     def distance_to_current_waypoint(self):
#         """
#         Gets distance in metres to the current waypoint.
#         It returns None for the first waypoint (Home location).
#         """
#         next_wp = AVIDRONE.commands.next
#         if next_wp == 0:
#             return None
#         mission_item = AVIDRONE.commands[next_wp - 1]  # commands are zero indexed
#         lat = mission_item.x
#         lon = mission_item.y
#         alt = mission_item.z
#         targetWaypointLocation = LocationGlobalRelative(lat, lon, alt)
#         distance_to_point = get_distance_metres(
#             AVIDRONE.location.global_frame, targetWaypointLocation
#         )
#         return distance_to_point


# def save_to_file(text_file, totalAlt, width, dLength, totalLength):
#     print("adding takeoff to altitude ", ALTITUDE)
#     AVIDRONE.commands.add(
#         Command(
#             0,
#             0,
#             0,
#             mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#             mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
#             0,
#             0,
#             0,
#             0,
#             0,
#             0,
#             0,
#             0,
#             0,
#         )
#     )
#     print("adding mission")
#     my_angle = 360 - np.degrees(AVIDRONE.attitude.yaw)
#     # TODO call Primary(search).rectangular
#     # if totalAlt == 0:
#     #     rectangular_primary_search_basic(
#     #         aviDrone.location.global_frame, width, dLength, totalLength, my_angle
#     #     )
#     # else:
#     #     rectangular_primary_search_with_alt(
#     #         aviDrone.location.global_frame,
#     #         width,
#     #         dLength,
#     #         totalLength,
#     #         totalAlt,
#     #         my_angle,
#     #     )
#     print("returning to launch")
#     # return_to_launch()
#     primary_functions.save_mission(text_file)
#     AVIDRONE.commands.clear()
#     print("Mission saved")


# # Search superclass
# class Search:
#     def __init__(self):
#         avidrone = drone.vehicle
#         self.global_frame = avidrone.location.global_frame

#     def get_distance(self, location_a, location_b):
#         lat_a, lat_b = location_a.lat, location_b.lat
#         lon_a, lon_b = location_a.lat, location_b.lat

#         d_lat = lat_b - lat_a
#         d_lon = lon_b - lon_a

#         return math.sqrt(math.pow(d_lat, 2) + math.pow(d_lon, 2)) * 1.113195e5

#     # TODO add utm/lat-long conversion functions here

#     def get_location(self, original_location, d_north, d_east, d_alt=0):
#         earth_radius = 6378137.0  # Radius of "spherical" earth
#         # Coordinate offsets in radians
#         d_lat = d_north / earth_radius
#         d_lon = d_east / (
#             earth_radius * math.cos(math.pi * original_location.lat / 180)
#         )

#         # New position in decimal degrees
#         new_lat = original_location.lat + (d_lat * 180 / math.pi)
#         new_lon = original_location.lon + (d_lon * 180 / math.pi)

#         if original_location.alt:
#             return LocationGlobal(new_lat, new_lon, original_location.alt)
#         else:
#             return LocationGlobal(new_lat, new_lon, d_alt)

#     def get_global_pos(self):
#         return self.global_frame

#     def mock_transceiver(self, uav_pos, beacon_pos):
#         mock_beacon = Transceiver.mock_transceiver(uav_pos, beacon_pos)
#         return mock_beacon


# #  Primary Search subclass
# class Primary(Search):
#     def __init__(self):
#         self.commands = AVIDRONE.commands
#         self.start_wp = Command(
#             0,
#             0,
#             0,
#             mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#             mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
#             0,
#             0,
#             0,
#             0,
#             0,
#             0,
#             0,
#             0,
#             ALTITUDE,
#         )
#         self.max_range = int(get_range(-1, -1))  # TODO get from parent search class

#     def search(self, len, d_len):
#         end_reached = False
#         stopping_point = 0
#         ENABLE_PRIMARY_SEARCH = True

#         if AVIDRONE.mode != "AUTO":
#             time.sleep(1)

#         while ENABLE_PRIMARY_SEARCH:
#             # TODO log battery info
#             next_wp = AVIDRONE.commands.next
#             # TODO log distance to waypoint

#             if mission.break_condition():
#                 time.sleep(1)
#                 AVIDRONE.commands.clear()
#                 AVIDRONE.commands.upload()
#                 time.sleep(1)
#                 stopping_point = next_wp
#                 break

#             if next_wp == get_range(len, d_len):
#                 end_reached = True
#                 # TODO log distance to waypoint

#                 stopping_point = next_wp
#                 break
#             time.sleep(1)
#         return end_reached, stopping_point

#     def rectangular(self, angle, location, width, length, height=0):
#         commands = []
#         _commands = self.commands
#         max_range = self.max_range
#         v_dist = h_dist = step = 0
#         arr = []

#         _commands.clear()  # Clears any existing commands
#         _commands.add(self.start_wp)
#         for _ in range(1, max_range):
#             if step == 0:
#                 h_dist = 0
#             elif (step == 1) or (step == 3):
#                 v_dist += length
#             else:
#                 h_dist = width
#             step += 1
#             step = step % 4
#             arr.append([h_dist, v_dist, 0])

#         # Rotation
#         initial_vector = (arr[1][0], arr[1][1], 0)
#         initial_vector = np.asarray(initial_vector)
#         final_vector = Vector.rotate_vector(initial_vector, angle)

#         if np.array_equal(final_vector, initial_vector):
#             # do not rotate if equal
#             rotated_vector = np.array(arr)
#         else:
#             # otherwise, rotate
#             rotated_vector = Vector.rotate_cloud(arr, initial_vector, final_vector)

#         for points in rotated_vector:
#             point = get_location_metres_with_alt(
#                 location, points[1], points[0], points[2]
#             )
#             wp_command = Command(
#                 0,
#                 0,
#                 0,
#                 mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#                 mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
#                 0,
#                 0,
#                 0,
#                 0,
#                 0,
#                 0,
#                 point.lat,
#                 point.lon,
#                 ALTITUDE,
#             )
#             _commands.add(wp_command)
#             commands.append(_commands)

#         if height == 0:
#             # add dummy waypoint at final point (lets us know when have reached destination)
#             final_wp = Command(
#                 0,
#                 0,
#                 0,
#                 mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#                 mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
#                 0,
#                 0,
#                 0,
#                 0,
#                 0,
#                 0,
#                 point.lat,
#                 point.lon,
#                 point.alt,
#             )

#         else:
#             # add dummy waypoint at final point (lets us know when have reached destination)
#             final_wp = Command(
#                 0,
#                 0,
#                 0,
#                 mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#                 mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
#                 0,
#                 0,
#                 0,
#                 0,
#                 0,
#                 0,
#                 point.lat,
#                 point.lon,
#                 0,
#             )
#         _commands.add(final_wp)
#         commands.append(_commands)
#         log.info(" Upload new commands to vehicle")
#         _commands.upload()


# #  Primary Search subclass
# class Secondary(Search):
#     # TODO Remove this comment: implement
#     def __init__(self):
#         pass

#     def search(self):
#         pass


# mission = Mission()  # TODO, move to right place





# def get_distance_metres(a_location1, a_location2):
#     """
#     Returns the ground distance in metres between two LocationGlobal objects.
#     This method is an approximation, and will not be accurate over large distances and close to the
#     earth's poles. It comes from the ArduPilot test code:
#     https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
#     """
#     d_lat = a_location2.lat - a_location1.lat
#     d_long = a_location2.lon - a_location1.lon
#     return math.sqrt((d_lat * d_lat) + (d_long * d_long)) * 1.113195e5


# def get_location_metres(original_location, dNorth, dEast):
#     """
#     Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
#     specified `original_location`. The returned Location has the same `alt` value
#     as `original_location`.
#     The function is useful when you want to move the vehicle around specifying locations relative to
#     the current vehicle position.
#     The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
#     For more information see:
#     http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
#     """
#     earth_radius = 6378137.0  # Radius of "spherical" earth
#     # Coordinate offsets in radians
#     dLat = dNorth / earth_radius
#     dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

#     # New position in decimal degrees
#     new_lat = original_location.lat + (dLat * 180 / math.pi)
#     new_lon = original_location.lon + (dLon * 180 / math.pi)
#     return LocationGlobal(new_lat, new_lon, original_location.alt)


# def get_location_metres_with_alt(original_location, dNorth, dEast, newAlt):
#     """
#     Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
#     specified `original_location`. The returned Location has the same `alt` value
#     as `original_location`.
#     The function is useful when you want to move the vehicle around specifying locations relative to
#     the current vehicle position.
#     The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
#     For more information see:
#     http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
#     """
#     earth_radius = 6378137.0  # Radius of "spherical" earth
#     # Coordinate offsets in radians
#     dLat = dNorth / earth_radius
#     dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

#     # New position in decimal degrees
#     new_lat = original_location.lat + (dLat * 180 / math.pi)
#     new_lon = original_location.lon + (dLon * 180 / math.pi)

#     return LocationGlobal(new_lat, new_lon, newAlt)





# def print_parameters():
#     print("\nPrint all parameters (iterate `aviDrone.parameters`):")
#     for key, value in AVIDRONE.parameters.items():
#         print(" Key:%s Value:%s" % (key, value))


# def battery_information():
#     print("Level:", AVIDRONE.battery.level)
#     print("Voltage:", AVIDRONE.battery.voltage)
#     print("Current:", AVIDRONE.battery.current)
#     print("Battery:", AVIDRONE.battery)
