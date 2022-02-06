#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import dronekit_sitl
from dronekit import connect

print("Start simulator (SITL)")
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()

# Connect to the Vehicle. (in this case a simulator running the same computer)
print("Connecting to vehicle on: %s" % (connection_string,))
avidrone = connect(connection_string, wait_ready=True)

sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()



def arm_and_takeoff(a_target_altitude):
    """
    Arms avidrone and fly to a_target_altitude.
    Taken from https://dronekit-python.readthedocs.io/en/latest/guide/taking_off.html
    """

    print("Basic pre-arm checks")  # Don't try to arm until autopilot is ready
    while not avidrone.is_armable:
        print(" Waiting for avidrone to initialize...")
        time.sleep(1)

    print("Arming motors")
    avidrone.mode = VehicleMode("GUIDED")  # UAV should arm in GUIDED mode
    avidrone.armed = True

    while not avidrone.armed:
        print(
            " Waiting for arming..."
        )  # Confirm avidrone armed before attempting to take off
        time.sleep(1)

    print("Taking off!")
    avidrone.simple_takeoff(a_target_altitude)  # Take off to target altitude

    # Wait until the avidrone reaches a safe height before processing the goto
    # (otherwise the command after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", avidrone.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if avidrone.location.global_relative_frame.alt >= a_target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


# Close vehicle object before exiting script
avidrone.close()

# Shut down simulator
sitl.stop()
print("Completed")


def param():
    # TODO rename file
    MAGNITUDE = 1  # Param 0, Set the distance the drone goes
    HEIGHT = 4  # Param 1, Set the height of the flight path
    DEGREES = 10  # Param 2,Set the amount to rotate in yaw
    DEGREE_ERROR = 2  # Param 3, Number of degrees error for rotation
    DISTANCE_ERROR = 0.35  # Param 4, Error in distance before target reached
    LAND_THRESHOLD = 2  # Param 5, Error in distance before target reached
    WINDOW_SIZE = 5  # Param 6, Set the size of the gps window original: 5

    param = [
        MAGNITUDE,
        HEIGHT,
        DEGREES,
        DEGREE_ERROR,
        DISTANCE_ERROR,
        LAND_THRESHOLD,
        WINDOW_SIZE,
    ]
    return param