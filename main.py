#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import logging as log
import os

import avidrone.search.util as util

if __name__ == "__main__":
    """
    Run searches as simple command line arguments.
    """
    parser = argparse.ArgumentParser()
    group = parser.add_argument_group()


    # define command line arguments
    group.add_argument("--primary", action="store_true", help="primary search only")
    group.add_argument("--v38", action="store_true", help="ensure python 3.8 runs")
    group.add_argument("--sim", action="store_true", help="run in simulated mode")
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


    # Arguments are not mutually exclusive so they are very explicit 
    # Example commands are shown in comments below. May need to be modified depending on python version & environment.
    # sim assumes udp connection on 127.0.0.1:14551
    # Make sure to run from proper directory & to install necessary packages
    # dronekit may not work with recent Python versions; our code at least works with Python 3.8.9
    if args.primary and not args.v38 and not args.sim and not args.secondary:
        os.system("python avidrone/search/primary.py")          # python main.py --primary

    elif args.primary and not args.v38 and args.sim and not args.secondary:
        os.system("python avidrone/search/primary.py --connect udp:127.0.0.1:14551") # python main.py --primary --sim

    elif args.primary and args.v38 and not args.sim and not args.secondary:
        os.system("py -3.8 avidrone/search/primary.py")         # py -3.8 main.py --primary --v38

    elif args.primary and args.v38 and args.sim and not args.secondary:
        os.system("py -3.8 avidrone/search/primary.py --connect udp:127.0.0.1:14551")   # py -3.8 main.py --primary --v38 --sim

    elif args.secondary and not args.primary:
        os.system("python3 avidrone/search/secondary.py")       # python3 main.py --secondary

    else:
        os.system("python3 avidrone/search/search.py")          # python3 main.py
