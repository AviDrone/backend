#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
import time

from mavsdk import System
from transceiver.direction_distance import directionDistance as Transceiver
from UAV.gps_data import GPSData
from UAV.parameter import parameter


async def secondary_search():
    """
    TODO make more modular
    For more info on flight modes:
    http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/plugins/telemetry.html
    """
    parameters = parameter()
    default_magnitude = parameters[0]
    default_altitude = parameters[1]
    default_window_size = parameters[6]
    default_land_threshold = parameters[5]

    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0")
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("Global position estimate ok")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off to altitude: ", default_altitude)
    await drone.action.set_takeoff_altitude(default_altitude)
    await drone.action.takeoff()

    """
    Secondary search mission
    """

    print("-- Initializing the gps_window")  # to be default_window_size long
    gps_window = GPSData(default_window_size)

    print("-- Setting STABILIZED flight mode")  # STABILIZED flight mode is 13
    await drone.telemetry.set_flight_mode(13)  # Originally GUIDED mode

    while drone.telemetry.flight_mode() == 13:
        print(
            "Direction: ",
            Transceiver.direction,
            "Distance: ",
            Transceiver.distance,
        )

        if Transceiver.direction < 2:
            # Turn left
            pass

        elif Transceiver.direction > 2:
            # Turn right
            pass

        elif Transceiver.direction == 2:
            print("Fly forward")
            # print flight direction
            global_position = drone.telemetry.position()
            gps_window.add_point(global_position, Transceiver.distance)

            if (
                gps_window.get_minimum_index() == ((gps_window.window_size - 1) / 2)
                and len(gps_window.gps_points) == gps_window.window_size
            ):

                # If the minimum is the center point of the gps_window we need to go
                # back to that location
                print("Min index = middle")

                # TODO REFACTOR: simple_goto_wait(gps_window.gps_points
                #  [int((gps_window.window_size - 1) / 2)])

                if gps_window.distance[2] <= default_land_threshold:
                    print("-- Landing")  # Land over local minimum
                    await drone.action.land()
                else:
                    print("Not close, continuing")
                    gps_window.purge_gps_window()
            elif (
                gps_window.get_minimum_index() == (gps_window.window_size - 1)
                and len(gps_window.gps_points) == gps_window.window_size
            ):

                # If the minimum data point is the last one in the array we have gone
                # too far and in the wrong direction
                print("Min index = window_size - 1")

                # TODO REFACTOR: condition_yaw(180, True)
                # TODO REFACTOR: simple_goto_wait(gps_window.gps_points[gps_window.window_size - 1])
                gps_window.purge_gps_window()

            elif gps_window.get_minimum_index() == 0:

                # If the minimum data point is in the first index, continue forward
                print("Min index = 0")

                # TODO REFACTOR: better_goto(default_magnitude, vehicle.attitude.yaw, vehicle)

            else:

                # Possibly going in the wrong direction.... But we still need to keep
                # going to make sure

                print("Goin' on up")
                # TODO REFACTOR: better_goto(default_magnitude, vehicle.attitude.yaw, vehicle)

            time.sleep(2)

    # TODO Test function


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(secondary_search())
