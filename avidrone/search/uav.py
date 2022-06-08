#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    UAV
"""
from __future__ import print_function

import argparse
import logging
import os
import logging

import dronekit_sitl
from dronekit import connect

# logging
log = logging.getLogger(__name__)
log.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s  [%(levelname)s]  %(message)s")
file_handler = logging.FileHandler(os.path.join("log", "uav.log"))
file_handler.setFormatter(formatter)
log.addHandler(file_handler)

# Connection string
parser = argparse.ArgumentParser(description="Connect to vehicle.")
parser.add_argument("--connect", help="Using a connection string, connect to Avidrone")
args = parser.parse_args()
connection_string = args.connect
sitl = dronekit_sitl.start_default()
connection_string_sitl = sitl.connection_string()

# Start SITL if no connection string specified
if not connection_string:
    connection_string = connection_string_sitl

class UAV:
    def __init__(self):
        self.id = 0
        self.nickname = 'Major Tom'
        self.connection_string = connection_string
        self.quad = connect(connection_string, wait_ready=True)


AVIDRONE = UAV() # Singleton

# Connect to the vehicle
log.debug("Connecting to vehicle on: %s", connection_string)