#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os

import avidrone.search.default_parameters as parameters

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()

    group.add_argument("--primary", action="store_true", help="primary search only")
    group.add_argument("--secondary", action="store_true", help="secondary search only")
    parser.add_argument(
        "-s", "--search", action="store_true", help="start avidrone search mission"
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="increase output verbosity"
    )

    args = parser.parse_args()

    print("\n********\nAVIDRONE\n********\n")
    if args.verbose:
        print("--Verbose: True")
        parameters.IS_VERBOSE = True

    if args.primary:
        os.system("python avidrone/search/primary_search.py")

    elif args.secondary:
        os.system("python3 avidrone/search/secondary_search.py")

    else:
        os.system("python3 avidrone/search/search.py")
