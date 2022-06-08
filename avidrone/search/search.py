#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    SEARCH
"""
import logging
import os

# Logging
log = logging.getLogger(__name__)
log.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
file_handler = logging.FileHandler(os.path.join("log", "search.log"))
file_handler.setFormatter(formatter)
log.addHandler(file_handler)

log.info("**************** SEARCH ****************")


class Search:
    def __init__(self):
        self.width = 50
        self.length = 100
        self.height = 0
        self.phase = "primary"  # TODO implement state machine here
