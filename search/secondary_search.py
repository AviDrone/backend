#!/usr/bin/env python3
# -*- coding: utf-8 -*-

<<<<<<< HEAD
# TODO implement secondary search


def secondary_search():
    pass
=======
import asyncio
import datetime
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
from transceiver.direction_distance import directionDistance as Transceiver
from UAV.gps_data import GPSData
from UAV.parameter import flight_mode, parameter

signal_found = False
parameters, flight_modes = parameter(), flight_mode()
default_magnitude = parameters[0]
default_altitude = parameters[1]
default_land_threshold = parameters[5]
default_window_size = parameters[6]

STABILIZED = flight_modes[12]


async def secondary_search():
    """
    Source:
    https://github.com/mavlink/MAVSDK-Python/blob/main/examples/offboard_position_ned.py
    """

    # Initiation sequence

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

    print("-- Setting initial set point")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(
            "Starting offboard." f" mode failed with error code: {error._result.result}"
        )
        print("-- Disarming")
        await drone.action.disarm()
        return

    print("-- Taking off to altitude (m): ", default_altitude, "\n")
    await drone.action.set_takeoff_altitude(default_altitude)
    await drone.action.takeoff()
    # Initiation sequence

    print("-- Initializing secondary search mission", "\n")
    print("-- Initializing the gps_window")  # to be default_window_size long
    gps_window = GPSData(default_window_size)

    print("-- Setting STABILIZED flight mode")
    await drone.telemetry.set_flight_mode(STABILIZED)  # Originally GUIDED mode

    while drone.telemetry.flight_mode() == STABILIZED:

        if Transceiver.direction < 2:  # Turn left
            print("-- Turn to face East")
            await drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, 0.0, -90.0)  # North  # East  # Down  # South
            )
            await asyncio.sleep(5)

        elif Transceiver.direction > 2:  # Turn right
            print("-- Turn to face West")
            await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, -90.0))
            await asyncio.sleep(5)

        elif Transceiver.direction == 2:  # Continue forward
            print("Fly forward")
            global_position = drone.telemetry.position()
            gps_window.add_point(global_position, Transceiver.distance)

            center_point = (gps_window.window_size - 1) / 2
            last_point = gps_window.window_size - 1

            # If the minimum is the center point of the gps_window,
            # we need to go back to that location

            if (
                gps_window.get_minimum_index() == center_point
                and len(gps_window.gps_points) == gps_window.window_size
            ):

                #  TODO IMPLEMENT
                #   drone.action.goto_location()

                if (
                    gps_window.distance[2]
                    <= default_land_threshold  # Land over local minimum
                ):
                    signal_found = True

                    print("-- Landing")
                    await drone.action.land()

                    if signal_found:
                        current_time = datetime.datetime.now()

                        print("--- SIGNAL FOUND --- ", f"-- time: {current_time}")
                        print(f"-- location: {global_position}")

                else:
                    print("Not close, continuing")
                    gps_window.purge_gps_window()

            elif (
                gps_window.get_minimum_index() == last_point
                and len(gps_window.gps_points) == gps_window.window_size
            ):

                # If the minimum data point is
                # the last one in the array we have gone

                print("too far, and in the wrong direction")

                # TODO REFACTOR
                #  condition_yaw(180, True)
                # TODO REFACTOR
                #  simple_goto_wait(
                #  gps_window.gps_points[gps_window.window_size - 1]
                #  )
                gps_window.purge_gps_window()

            elif gps_window.get_minimum_index() == 0:

                # If the minimum data point is in the first index,
                print("continue forward")

                # TODO REFACTOR
                #  better_goto(
                #  default_magnitude,
                #  vehicle.attitude.yaw
                #  , vehicle
                #  )

            else:
                print(f"Did not find signal at altitude: {default_altitude}")
                print("Climbing...")
                # TODO REFACTOR
                #  better_goto(
                #  default_magnitude,
                #  vehicle.attitude.yaw,
                #  vehicle
                #  )

        time.sleep(2)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(
            "Stopping offboard. ",
            f"mode failed with error code: {error._result.result}",
        )


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(secondary_search())
>>>>>>> cb7f8bcdbfa52b56a9cf9c4d100d32f0f54aedaf
