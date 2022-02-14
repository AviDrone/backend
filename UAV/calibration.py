#!/usr/bin/env python3

import asyncio

from mavsdk import System

# TODO test this with our own drone.


<<<<<<< HEAD
async def run():
=======
async def calibrate():
>>>>>>> cb7f8bcdbfa52b56a9cf9c4d100d32f0f54aedaf
    """
    For more info, see:
    http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/plugins/calibration.html
    https://github.com/mavlink/MAVSDK-Python/blob/main/examples/calibration.py

    """

    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("-- Starting gyroscope calibration")
    async for progress_data in drone.calibration.calibrate_gyro():
        print(progress_data)
    print("-- Gyroscope calibration finished")

    print("-- Starting accelerometer calibration")
    async for progress_data in drone.calibration.calibrate_accelerometer():
        print(progress_data)
    print("-- Accelerometer calibration finished")

    print("-- Starting magnetometer calibration")
    async for progress_data in drone.calibration.calibrate_magnetometer():
        print(progress_data)
    print("-- Magnetometer calibration finished")

    print("-- Starting board level horizon calibration")
    async for progress_data in drone.calibration.calibrate_level_horizon():
        print(progress_data)
    print("-- Board level calibration finished")


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
<<<<<<< HEAD
    loop.run_until_complete(run())
=======
    loop.run_until_complete(calibrate())
>>>>>>> cb7f8bcdbfa52b56a9cf9c4d100d32f0f54aedaf
