#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    TRANSCEIVER
    TODO refactor so that this cannot be run alone
"""

import ctypes
import datetime
import logging
import math
import pathlib
import random
import time

import numpy as np

# logging
log = logging.getLogger(__name__)
log.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s  [%(levelname)s]  %(message)s")
file_handler = logging.FileHandler("transceiver.log")
file_handler.setFormatter(formatter)
log.addHandler(file_handler)

IS_TEST = True  # set to true to use mock transceiver simulation
IS_TIMEOUT = False
timeout_count = 0


class Transceiver:
    """
    # Transceiver model reference: https://beaconreviews.com/search_strip_widths.php
    # Battery life from: https://www.backcountry.com/backcountry-access-tracker-dts-beacon#
    """

    def __init__(self, id):
        # Settings
        self.id = id  # Avidrone
        self.mode = "transmit"  # or receive
        self.model = {
            0: "Avidrone",
            1: "Tracker DTS",
            2: "Orthovox Zoom+",
            3: "Orthovox 3+",
            4: "Tracker2",
            5: "Tracker3",
            6: "Pieps DSP Sport",
            7: "Pieps DSP Pro",
            8: "Mammut Pulse",
            9: "Orthovox S1",
        }
        self.search_strip_width = {
            "Avidrone": 20,
            "Tracker DTS": 34,
            "Orthovox Zoom+": 31,
            "Orthovox 3+": 36,
            "Tracker2": 38,
            "Tracker3": 39,
            "Pieps DSP Sport": 41,
            "Pieps DSP Pro": 42,
            "Mammut Pulse": 54,
            "Orthovox S1": 50,
        }
        self.curr_model = self.model[self.id]
        self.curr_search_strip_width = self.search_strip_width[self.curr_model]

        # Transceiver data
        self.direction = -1  # not detected
        self.distance = -1  # not detected
        self.signal_detected = False  # not detected

        # Beacon position
        self.beacon_x = None
        self.beacon_y = None
        self.beacon_z = None
        self.position = [self.beacon_x, self.beacon_y, self.beacon_z]

        # Simulation
        self._frequency = 457  # mHz
        self._coil_number = 2  # DTS transceiver
        self._coil_length = 120  # mm
        self._coil_current = 750  # AAA Battery power source
        self._coil_angle_offset = 45  # degrees
        # seconds in battery at 100%
        self._battery = 720000 if self.mode == "transmit" else 180000

    def read_transceiver(self):
        shared_ctypes_lib = (
            pathlib.Path().absolute() / "read_transceiver.lib"
        )  # TODO implement this
        c_lib = ctypes.CDLL(shared_ctypes_lib)
        output_array = (ctypes.c_double * 2)()
        c_lib._Z11get_dir_digPd(ctypes.byref(output_array))
        log.debug(f"direction, distance: {output_array[0],output_array[1]}")

        transceiver_output = [output_array[0], output_array[1]]
        return transceiver_output

    def get_model_info(self):
        model_info = [self.id, self.curr_model, self.curr_search_strip_width]
        return model_info

    def switch_mode(self):
        if self.mode == "transmit":
            self.mode = "receive"
        elif self.mode == "receive":
            self.mode = "transmit"
        else:
            log.warning("Unknown mode")

    def signal_found_msg(self):
        print(f"\n-------- SIGNAL FOUND: {TRANSCEIVER.signal_detected} -------- ")
        print(f"-- Current ime: {current_time}")
        print("-- Mission time: ", mission_time)
        print(f"-- Location: {TRANSCEIVER.position}\n")

    def signal_not_found_msg(self):
        print(f"\n-------- SIGNAL FOUND: {TRANSCEIVER.signal_detected} -------- ")
        print(f"-- Current ime: {current_time}")
        print(f"-- Mission time: {mission_time}")

    def show_data(self):
        data_msg = f" --- transceiver data --- \n \
        transceiver.mode: {self.mode} \n \
        transceiver.direction: {self.direction} \n \
        transceiver.distance: {self.distance} \n \
        transceiver.signal_detected: {self.signal_detected} \n"
        print(data_msg)

    def show_settings(self):
        settings_msg = f" --- settings --- \n \
            transceiver.mode: {self.mode} \n \
            transceiver.model: {self.model} \n \
            transceiver.search_strip_width: {self.curr_search_strip_width}"
        print(settings_msg)

    def mock_transceiver(self, uav_pos, beacon_pos):
        # UAV position
        x_1 = uav_pos[0]
        y_1 = uav_pos[1]
        z_1 = uav_pos[2]

        self.beacon_x = beacon_pos[0]
        self.beacon_x = beacon_pos[0]
        self.beacon_x = beacon_pos[0]

        displacement = self.get_displacement(
            x_1, self.beacon_x, y_1, self.beacon_y, z_1, self.beacon_z
        )
        disp_n = self.normalize(displacement)
        distance = self.get_distance(displacement)
        theta = self.get_theta(disp_n)
        direction = self.get_direction(theta)
        return direction, distance

    def get_displacement(self, x_1, x_2, y_1, y_2, z_1=0, z_2=0):
        displacement = [x_2 - x_1, y_2 - y_1, z_2 - z_1]
        log.debug(f"displacement: {displacement}")
        return displacement

    def normalize(self, disp):
        d_v = disp / np.linalg.norm(disp)
        d_v_normal = d_v.tolist()
        log.debug(f"d_v_normal: {d_v_normal}")
        return d_v_normal

    def get_distance(self, disp):
        if disp[2] == 0:
            distance = math.sqrt((disp[0] ** 2 + disp[1] ** 2))
        else:
            distance = math.sqrt((disp[0] ** 2 + disp[1] ** 2 + disp[2] ** 2))
        log.debug(f"distance_xyz: {distance}")
        return distance

    def get_theta(self, disp):
        fwd = [1, 0]  # UAV's forward vector.
        v_d = [disp[0], disp[1]]
        d_xy = self.get_distance(disp)

        if d_xy != 0:
            theta = np.arccos(np.dot(v_d, fwd) / d_xy)

        else:
            d_xy = 0.001  # to avoid division by 0
            theta = np.arccos(np.dot(v_d, fwd) / d_xy)

        # To account for measurement inconsistencies. We use a random value
        # between -15 and 15. That makes it likely that the beacon gets the
        # wrong direction roughly a third of the time.

        theta_random = theta + random.uniform(-15, 15)
        log.debug(f"theta + random.uniform(-15, 15): {theta + random.uniform(-15, 15)}")
        return theta_random

    def get_direction(self, theta):
        """
        led 0: Theta -90 to -30
        led 1: Theta -30 to -10
        led 2: Theta -10 to +10
        led 3: Theta +10 to +30
        led 4: Theta +30 to +90

        """

        direction = -1  # direction not acquired

        if theta < -30:
            log.debug(direction)
            direction = 0

        elif -30 <= theta < -10:
            log.debug(direction)
            direction = 1

        elif -10 <= theta < 10:
            log.debug(direction)
            direction = 1

        elif 10 <= theta < 30:
            log.debug(direction)
            direction = 2

        elif 30 <= theta < 90:
            log.debug(direction)
            direction = 3

        elif theta >= 90:
            log.debug(direction)
            direction = 4

        return direction


TRANSCEIVER = Transceiver(0)  # Avidrone model


uav_pos = [120, 10, 20]  # Example
beacon_pos = [20, 20, 2]  # Example


# Mock beacon
mock_beacon = TRANSCEIVER.mock_transceiver(uav_pos, beacon_pos)
run = False
while run:
    timeout_count += 1

    if timeout_count == TRANSCEIVER._battery:
        IS_TIMEOUT = True

    log.info(f"-- direction, distance: {(TRANSCEIVER.direction, TRANSCEIVER.distance)}")
    mission_begin_time = datetime.datetime.now()

    if uav_pos[0] == beacon_pos[0] and uav_pos[1] == beacon_pos[1]:
        TRANSCEIVER.signal_detected = True

        if TRANSCEIVER.signal_detected:
            uav_pos[2] -= 1

            if uav_pos[2] == 0:
                curr_t = datetime.datetime.now()
                current_time = curr_t.strftime("%c")

                mission_end_time = datetime.datetime.now()
                mission_time = mission_end_time - mission_begin_time
                TRANSCEIVER.position = uav_pos
                TRANSCEIVER.signal_found_msg()
                break

    # TODO this should happen in secondary
    # Navigation algorithm
    if uav_pos[0] < beacon_pos[0]:  # x
        uav_pos[0] += 1
        log.debug("x_uav < x_beacon")

    elif uav_pos[1] < beacon_pos[1]:  # y
        uav_pos[1] += 1
        log.debug("y_uav < y_beacon")

    if uav_pos[0] > beacon_pos[0]:  # y
        uav_pos[0] -= 1
        log.debug("y_uav > y_beacon")

    elif uav_pos[1] > beacon_pos[1]:  # x
        uav_pos[1] -= 1
        log.debug("x_uav > x_beacon")

    else:
        time.sleep(0.0)  # Beacon reads values every 0.5 seconds

    if IS_TIMEOUT:
        log.warning("\n reached timeout \n")
        current_time = str(datetime.datetime.now())
        mission_end_time = datetime.datetime.now()
        mission_time = mission_end_time - mission_begin_time
        TRANSCEIVER.signal_not_found_msg()
        break
