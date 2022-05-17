#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    AVIDRONE SEARCH
"""
from __future__ import print_function

import argparse
import asyncio
import datetime
import logging as log
import math
import time

import primary
import secondary
import util
from dronekit import (
    Command,
    LocationGlobal,
    LocationGlobalRelative,
    VehicleMode,
    connect,
)


def search():
    log.info(f"-- default size: {util.DEGREES}")
    log.info(f"-- default altitude: {util.ALTITUDE}")
    log.info(f"-- default land threshold: {util.LAND_THRESHOLD}")


if __name__ == "__main__":
    search()
