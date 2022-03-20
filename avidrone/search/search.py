#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import print_function

import argparse
import asyncio
import datetime
import math
import time

import primary_search as primary_search
import secondary_search as secondary_search
from dronekit import (
    Command,
    LocationGlobal,
    LocationGlobalRelative,
    VehicleMode,
    connect,
)
import default_parameters as default
import secondary


def run():
    print(f"-- default size: {default.DEGREES}")
    print(f"-- default altitude: {default.ALTITUDE}")
    print(f"-- default land threshold: {default.LAND_THRESHOLD} \n")

    secondary.run()


if __name__ == "__main__":
    run()
