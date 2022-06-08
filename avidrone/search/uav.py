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

# logging
log = logging.getLogger(__name__)
log.setLevel(logging.INFO)
msg = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
file_handler = logging.FileHandler(os.path.join("log", "uav.log"))
file_handler.setFormatter(msg)
log.addHandler(file_handler)

# Connection string
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


AVIDRONE = UAV()  # Singleton
