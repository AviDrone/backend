#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# For interfacing with the C code
import ctypes
import pathlib


class DirectionDistance:
    """
    The direction variable holds a number between 0-4 to represent from left to right which LED is lit.
    The distance variable holds a number the number corresponding to the distance displayed on the transceiver
    """

    def __init__(self, direction, distance):
        self.direction = direction
        self.distance = distance


def read_transceiver():
    shared_ctypes_lib = pathlib.Path().absolute() / "read_transceiver.lib"
    c_lib = ctypes.CDLL(shared_ctypes_lib)

    output_array = (ctypes.c_double * 2)()
    print("output_array before:", output_array[0], " ", output_array[1])
    c_lib._Z11get_dir_digPd(ctypes.byref(output_array))
    print(
        "Direction: ",
        output_array[0],
        "Distance: ",
        output_array[1],
    )
    return DirectionDistance(output_array[0], output_array[1])
