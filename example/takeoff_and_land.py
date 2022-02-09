#!/usr/bin/env python3

import asyncio

from mavsdk import System


async def run():
    """
        For more info, see:
    https://github.com/mavlink/MAVSDK-Python/blob/main/examples/takeoff_and_land.py
    """
    avidrone = System()
    await avidrone.connect(system_address="udp://:14540")

    print("Waiting for avidrone to connect...")
    async for state in avidrone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break

    print("Waiting for avidrone to have a global position estimate...")
    async for health in avidrone.telemetry.health():
        if health.is_global_position_ok:
            print("Global position estimate ok")
            break

    print("-- Arming")
    await avidrone.action.arm()

    print("-- Taking off")
    await avidrone.action.takeoff()

    await asyncio.sleep(5)

    print("-- Landing")
    await avidrone.action.land()


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
