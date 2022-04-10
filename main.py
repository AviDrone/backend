#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import logging as log
import os

import avidrone.search.util as util

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()

    group.add_argument("--primary", action="store_true", help="primary search only")
    group.add_argument("--secondary", action="store_true", help="secondary search only")
    parser.add_argument("-test", action="store_true", help="Testing search")
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="increase output verbosity"
    )
    parser.add_argument(
        "-search", action="store_true", help="start avidrone search mission"
    )
    args = parser.parse_args()

    print("\n********\nAvidrone Search\n********\n")
    if args.verbose:
        log.info("--Verbose: True")
        parameters.IS_VERBOSE = True

    if args.primary:
        os.system("python avidrone/search/primary.py")

    elif args.secondary:
        os.system("python3 avidrone/search/secondary.py")

    else:
        os.system("python3 avidrone/search/search.py")
