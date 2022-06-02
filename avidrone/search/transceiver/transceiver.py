#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    TRANSCEIVER
"""

import datetime
import logging
import time

from transceiver import EM_field, util

# logging
log = logging.getLogger(__name__)
log.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s  [%(levelname)s]  %(message)s")
file_handler = logging.FileHandler("transceiver.log")
file_handler.setFormatter(formatter)
log.addHandler(file_handler)


class Transceiver:
    def __init__(self):
        self.direction = -1  # initially not detected
        self.distance = -1  # initially not detected
        self.signal_detected = False  # not detected
        self.position = [0, 0, 0]  # Default example (1 meter underground)
        self.mode = "transmit"  # or detect
        self.model_number = 0  # Avidrone (default)

        self.battery = (
            720000 if self.mode == "transmit" else 180000  # seconds in battery at 100%
        )  # Battery life from: https://www.backcountry.com/backcountry-access-tracker-dts-beacon#

        self.model = [
            "Avidrone",
            "Tracker DTS",
            "Orthovox Zoom+",
            "Orthovox 3+",
            "Tracker2",
            "Tracker3",
            "Pieps DSP Sport",
            "Pieps DSP Pro",
            "Mammut Pulse",
            "Orthovox S1",
        ]  # Model reference: https://beaconreviews.com/search_strip_widths.php

        self.search_strip_width = [
            {"Avidrone": 6},
            {"Tracker DTS": 34},
            {"Orthovox Zoom+": 31},
            {"Orthovox 3+": 36},
            {"Tracker2": 38},
            {"Tracker3": 39},
            {"Pieps DSP Sport": 41},
            {"Pieps DSP Pro": 42},
            {"Mammut Pulse": 54},
            {"Orthovox S1": 50},
        ]

        self.curr_search_strip_width = 6  # Avidrone (default)

        # Simulation
        self.frequency_ = 457  # mHz
        self.coil_number_ = 2  # Avidrone (default)
        self.coil_length_ = 120  # mm
        self.coil_current_ = 750  # AAA Battery power source
        self.coil_angle_offset_ = 45  # degrees

    def get_model(self):
        return transceiver.model[self.model_number]

    def switch_mode(self):
        if self.mode == "transmit":
            self.mode = "receive"
        elif self.mode == "receive":
            self.mode = "transmit"
        else:
            log.warning("Unknown mode")

    def victim_found_msg(self):
        print(f"\n-------- VICTIM FOUND: {transceiver.signal_detected} -------- ")
        print(f"-- Current ime: {current_time}")
        print("-- Mission time: ", mission_time)
        print(f"-- Location: {transceiver.position}\n")

    def victim_not_found_msg(self):
        print(f"\n-------- VICTIM FOUND: {transceiver.signal_detected} -------- ")
        print(f"-- Current ime: {current_time}")
        print(f"-- Mission time: {mission_time}")

    @staticmethod
    def show_data():
        data_msg = f" --- transceiver data --- \n \
        transceiver.mode: {transceiver.mode} \n \
        transceiver.direction: {transceiver.direction} \n \
        transceiver.distance: {transceiver.distance} \n \
        transceiver.signal_detected: {transceiver.signal_detected} \n"
        print(data_msg)
        log.info(data_msg)

    @staticmethod
    def show_settings():
        settings_msg = f" --- settings --- \n \
            transceiver.mode: {transceiver.mode} \n \
            transceiver.model: {model} \n \
            transceiver.search_strip_width: {transceiver.curr_search_strip_width}"
        print(settings_msg)
        log.info(settings_msg)

    @staticmethod
    def mock_transceiver(uav_pos, beacon_pos):
        # UAV position
        x_1 = uav_pos[0]
        y_1 = uav_pos[1]
        z_1 = uav_pos[2]

        # Beacon position
        x_2 = beacon_pos[0]
        y_2 = beacon_pos[1]
        z_2 = beacon_pos[2]

        displacement = util.get_displacement(x_1, x_2, y_1, y_2, z_1, z_2)
        disp_n = util.normalize(displacement)
        dist = util.get_distance_xy(displacement)
        theta = util.get_unique_theta(disp_n)
        direction = util.get_direction(theta)
        return direction, dist


IS_TEST = True  # set to true to use mock transceiver simulation
IS_TIMEOUT = False
timeout_count = 0

# initialize transceiver parameters
transceiver = Transceiver()
transceiver.model_number = 2  # Avidrone
model = transceiver.get_model()
transceiver.curr_search_strip_width = transceiver.search_strip_width[
    transceiver.model_number
][model]


uav_pos = [10, 10, 20]  # Example
beacon_pos = transceiver.position


# Mock beacon
mock_beacon = transceiver.mock_transceiver(uav_pos, beacon_pos)

while True:
    timeout_count += 1

    if IS_TEST:
        transceiver.direction = mock_beacon[0]
        transceiver.distance = mock_beacon[1]

    else:
        print("Too bad!!")
        # transceiver.direction = int(transceiver.theta[timeout_count])
        # transceiver.distance = mock_transceiver[1]  # TODO replace with real values

    if timeout_count == transceiver.battery:
        IS_TIMEOUT = True

    log.info(f"-- direction, distance: {(transceiver.direction, transceiver.distance)}")
    mission_begin_time = datetime.datetime.now()

    if uav_pos[0] == beacon_pos[0] and uav_pos[1] == beacon_pos[1]:
        transceiver.signal_detected = True

        if transceiver.signal_detected:
            uav_pos[2] -= 1

            if uav_pos[2] == 0:
                curr_t = datetime.datetime.now()
                current_time = curr_t.strftime("%c")

                mission_end_time = datetime.datetime.now()
                mission_time = mission_end_time - mission_begin_time
                transceiver.position = uav_pos
                transceiver.victim_found_msg()
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
        if IS_TEST:
            time.sleep(0.0)  # To speed up search time during testing
        else:
            time.sleep(0.5)  # Beacon reads values every 0.5 seconds

        if IS_TIMEOUT:
            log.warning("\n reached timeout \n")
            current_time = datetime.datetime.now()
            mission_end_time = datetime.datetime.now()
            mission_time = mission_end_time - mission_begin_time
            transceiver.victim_not_found_msg()
            IS_TIMEOUT = True
            break
