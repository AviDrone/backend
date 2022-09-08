#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import datetime
import logging
import os
import time

from dronekit import VehicleMode
from parameters import MISSION_TIMEOUT
from search import SEARCH, SECONDARY
from transceiver.transceiver import TRANSCEIVER
from uav import AVIDRONE

SEARCH.is_enabled_secondary = True

log = logging.getLogger(__name__)
log.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
file_handler = logging.FileHandler(os.path.join("log", "secondary.log"))
file_handler.setFormatter(formatter)
log.addHandler(file_handler)

# parameters

log.debug(f"transceiver model: {TRANSCEIVER.curr_model}")
log.debug(f"search strip width: {TRANSCEIVER.curr_search_strip_width}")


if SEARCH.is_enabled_secondary:
    SECONDARY.is_enabled = True

log.debug(AVIDRONE.mode)
log.info("-- set GUIDED mode")
AVIDRONE.mode = VehicleMode("GUIDED")

assert SECONDARY.is_enabled
SECONDARY.search()
