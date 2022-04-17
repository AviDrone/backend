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

from dronekit import (
    Command,
    LocationGlobal,
    LocationGlobalRelative,
    VehicleMode,
    connect,
)

import primary
import secondary
import util


def search():
    log.info(f"-- default size: {util.DEGREES}")
    log.info(f"-- default altitude: {util.ALTITUDE}")
    log.info(f"-- default land threshold: {util.LAND_THRESHOLD} \n")

    """
    pseudocode
    
    while util.FLIGHT_MODE == "GUIDED":
        #  Run primary search until signal is consistently detected. 
        #  Otherwise, return "SIGNAL NOT FOUND" (error: signal_not_found)
        
        primary_search = primary.run()  
        
        # Once primary search detects a signal consistently, 
        # it returns two position vectors. 'uav_pos' and 'beacon_pos'.
        if primary_search == "COMPLETE":    
            secondary_search = secondary.run()
            
        if secondary_search == "COMPLETE":
            return signal_pos   # return value in either lat, long tuple or UTM.
            
    """
    # TODO integrate primary and secondary search to run continuously


if __name__ == "__main__":
    search()
