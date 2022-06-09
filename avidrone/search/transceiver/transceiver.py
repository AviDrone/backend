#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    TRANSCEIVER
"""

import ctypes
import datetime
import logging
import pathlib
import time

from util import (
    get_direction,
    get_displacement,
    get_distance_xy,
    get_theta,
    normalize,
)

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
        self.id = id # Avidrone
        self.mode = "transmit" # or receive 
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
        self.position = [46.045030, -118.3911911, 343]  # Roger's Field

        # Simulation
        self._frequency = 457  # mHz
        self._coil_number = 2  # DTS transceiver
        self._coil_length = 120  # mm
        self._coil_current = 750  # AAA Battery power source
        self._coil_angle_offset = 45  # degrees
        self._battery = (
            720000 if self.mode == "transmit" else 180000  # seconds in battery at 100%
        )

    def read_transceiver(self):
        shared_ctypes_lib = (
            pathlib.Path().absolute() / "read_transceiver.lib"
        )  # TODO implement this
        c_lib = ctypes.CDLL(shared_ctypes_lib)
        output_array = (ctypes.c_double * 2)()
        c_lib._Z11get_dir_digPd(ctypes.byref(output_array))
        log.debug(
            "Direction: ",
            output_array[0],
            "Distance: ",
            output_array[1],
        )

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

        # Beacon position
        x_2 = beacon_pos[0]
        y_2 = beacon_pos[1]
        z_2 = beacon_pos[2]

        displacement = get_displacement(x_1, x_2, y_1, y_2, z_1, z_2)
        disp_n = normalize(displacement)
        distance = get_distance_xy(displacement)
        theta = get_theta(disp_n)
        direction = get_direction(theta)
        return direction, distance


TRANSCEIVER = Transceiver(1)    # Singleton


uav_pos = [120, 10, 20]  # Example
beacon_pos = [20, 20, 2]  # Example



# Mock beacon
mock_beacon = TRANSCEIVER.mock_transceiver(uav_pos, beacon_pos)

while True:
    timeout_count += 1

    if IS_TEST:
        TRANSCEIVER.direction = mock_beacon[0]
        TRANSCEIVER.distance = mock_beacon[1]

    else:
        print("Too bad!!")  # TODO replace with real values
        # transceiver.direction = int(transceiver.theta[timeout_count])
        # transceiver.distance = mock_transceiver[1]

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
            current_time = datetime.datetime.now()
            mission_end_time = datetime.datetime.now()
            mission_time = mission_end_time - mission_begin_time
            TRANSCEIVER.signal_not_found_msg()
            IS_TIMEOUT = True
            break
