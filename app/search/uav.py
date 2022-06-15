#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    UAV
"""
from __future__ import print_function

import argparse
import logging
import os

import dronekit_sitl
from dronekit import connect

log = logging.getLogger(__name__)
log.setLevel(logging.INFO)
msg = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
file_handler = logging.FileHandler(os.path.join("log", "uav.log"))
file_handler.setFormatter(msg)
log.addHandler(file_handler)

parser = argparse.ArgumentParser(description="Connect to  a vehicle.")
parser.add_argument("--connect", help="Connection string to vehicle.")
args = parser.parse_args()
connection_str = args.connect

if not connection_str:
    sitl = dronekit_sitl.start_default()
    connection_string_sitl = sitl.connection_string()
    connection_str = connection_string_sitl

log.debug("Connecting to vehicle on: %s", connection_str)


class UAV:
    def __init__(self):
        self.id = 0
        self.nickname = "Major Tom"
        self.connection_string = connection_str
        self.quad = connect(self.connection_string, wait_ready=True)
        self.mode = self.quad.mode.name

        # Navigation
        self.location = self.quad.location.global_frame
        self.altitude = self.quad.location.global_relative_frame.alt

        self.yaw = self.quad.attitude.yaw
        self.angle = 360 - self.yaw

        # Missions
        self.mode = self.quad.mode
        self.commands = self.quad.commands

        # Settings
        self.parameters = self.quad.parameters
        self.enable_battery_telemetry = self.quad.parameters.set("FS_BATT_ENABLE", 2)
        self.battery = self.quad.battery

    def print_parameters(self):
        print("\nPrint all parameters: ")
        for key, value in AVIDRONE.parameters.items():
            print(" Key:%s Value:%s" % (key, value))

    def battery_information(self):
        log.info("Level:", AVIDRONE.battery.level)
        log.info("Voltage:", AVIDRONE.battery.voltage)
        log.info("Current:", AVIDRONE.battery.current)
        log.info("Battery:", AVIDRONE.battery)


AVIDRONE = UAV()
