#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
vehicle_state.py: 

Demonstrates how to get and set vehicle state and parameter information, 
and how to observe vehicle attribute (state) changes.

Full documentation is provided at http://python.dronekit.io/examples/vehicle_state.html

Source:
https://dronekit-python.readthedocs.io/en/latest/examples/vehicle_state.html
"""
from __future__ import print_function

import argparse
import time

import dronekit_sitl
from dronekit import VehicleMode, connect

parser = argparse.ArgumentParser(
    description="Print out vehicle state information. Connects to SITL on local PC by default."
)
parser.add_argument(
    "--connect",
    help="vehicle connection target string. If not specified, SITL automatically started and used.",
)
args = parser.parse_args()

connection_string = args.connect
sitl = None
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle.
#   Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.
print("\nConnecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=True)

vehicle.mode = VehicleMode("GUIDED")
print(" Armed: %s" % vehicle.armed)  # settable


print("\nSet Vehicle.mode = GUIDED (currently: %s)" % vehicle.mode.name)
while not vehicle.mode.name == "GUIDED":  # Wait until mode has changed
    print(" Waiting for mode change ...")
    time.sleep(1)
    
while vehicle.mode.name == "GUIDED":
    print(" Rangefinder: %s" % vehicle.rangefinder)
    print(" Rangefinder distance: %s" % vehicle.rangefinder.distance)
    print(" Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
