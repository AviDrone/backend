#!/usr/bin/env python


class DirectionDistance:
    """
    The direction variable holds a number between 0-4
    to represent from left to right which transceiver LED is lit.

    The distance variable holds a number the number corresponding
    to the distance displayed on the transceiver
    """

    def __init__(self, direction, distance):
        self.direction = direction
        self.distance = distance
