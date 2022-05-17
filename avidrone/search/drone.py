from __future__ import print_function

import argparse
import logging as log

from dronekit import connect
from pymavlink import mavutil
from util import WINDOW_SIZE, GpsData, Mission, Search, Vector

# init vehicle

parser = argparse.ArgumentParser(description="Demonstrates basic mission operations.")
parser.add_argument(
    "--connect",
    help="vehicle connection target string.If not specified, SITL started and used.",
)
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Connect to the vehicle
log.info("Connecting to vehicle on: %s", connection_string)
vehicle = connect(connection_string, wait_ready=True)

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl

    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# init
vector = Vector()
mission = Mission()
gps_data = GpsData(window_size=WINDOW_SIZE)
search = Search()
