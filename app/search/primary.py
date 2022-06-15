#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import logging
import os
import time

from dronekit import VehicleMode
from search import PRIMARY, SEARCH
from transceiver.transceiver import TRANSCEIVER
from uav import AVIDRONE

SEARCH.ENABLE_PRIMARY_SEARCH = True

log = logging.getLogger(__name__)
log.setLevel(logging.INFO)
msg = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
file_handler = logging.FileHandler(os.path.join("log", "primary.log"))
file_handler.setFormatter(msg)
log.addHandler(file_handler)

# parameters
width = SEARCH.width
height = SEARCH.height
length = SEARCH.length
strip_width = TRANSCEIVER.curr_search_strip_width

if SEARCH.ENABLE_PRIMARY_SEARCH:
    PRIMARY.is_enabled = True

log.debug(AVIDRONE.mode)
log.info("-- set GUIDED mode")
AVIDRONE.mode = VehicleMode("GUIDED")

# TODO refactor so it says PRIMARY.search
assert PRIMARY.is_enabled
PRIMARY.save_to_file(width, length, height)
