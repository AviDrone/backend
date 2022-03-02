#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import print_function

# Set up option parsing to get connection string
import argparse
import asyncio
import datetime
import math
import time

from dronekit import (
    Command,
    LocationGlobal,
    LocationGlobalRelative,
    VehicleMode,
    connect,
)
from pymavlink import mavutil
import primary_search as primary_search
import secondary_search as secondary_search


def run():
    # Assert the UAV is ready and armed
    parser = argparse.ArgumentParser(
        description="Demonstrates basic mission operations."
    )
    parser.add_argument(
        "--connect",
        help="vehicle connection target string. If not specified, SITL automatically started and used.",
    )
    args = parser.parse_args()

    connection_string = args.connect
    sitl = None

    # Start SITL if no connection string specified
    if not connection_string:
        import dronekit_sitl

        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    # Initiate primary search
    # run rectangular primary search
    # If signal is detected
        # End primary search
    # Assert primary search ended
    # Initiate secondary search
    # Run secondary search
    # If secondary search complete
        # Create message
        # send message using GUI
    # end secondary search
    # Assert search is complete
    print("Running search!!")  # TODO implement
