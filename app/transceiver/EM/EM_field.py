#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    EM field
"""

import math

import magpylib as magpy
import numpy as np

"""
# Simulation of electromagnetic field of transceiver coils
# For more information about the coils and units used, see: 
# https://magpylib.readthedocs.io/en/latest/examples/examples_30_coil_field_lines.html
# https://numpy.org/doc/stable/reference/generated/numpy.linspace.html
"""


class EMField:
    def __init__(self):
        self.home = [0, 0, 0]  # test
        self.current = 750  # Amps
        self.coil_length = 120  # Millimeters

        # create grid
        self.ts = np.linspace(-100000, 100000, 100)  # (DO NOT MODIFY)
        self.grid = np.array([[(x, 0, z) for x in self.ts] for z in self.ts])

        # create coil 1
        ts = np.linspace(-self.coil_length / 2, self.coil_length / 2, 1200)
        vertices_1 = np.c_[5 * np.cos(ts * 2 * np.pi), 5 * np.sin(ts * 2 * np.pi), ts]
        coil_1 = magpy.current.Line(current=self.current / 2, vertices=vertices_1)
        self.coil_1 = coil_1.rotate_from_angax(45, "y")  # Front coil

        # create coil 2
        ts = np.linspace(-self.coil_length / 2, self.coil_length / 2, 1200)
        vertices_2 = np.c_[5 * np.cos(ts * 2 * np.pi), 5 * np.sin(ts * 2 * np.pi), ts]
        coil_2 = magpy.current.Line(current=self.current / 2, vertices=vertices_2)
        self.coil_2 = coil_2.rotate_from_angax(-45, "y")  # Back coil

        # compute field of coil 1
        self.B_1 = magpy.getB(self.coil_1, self.grid)
        Bamp_1 = np.linalg.norm(self.B_1, axis=2)
        Bamp_1 /= np.amax(Bamp_1)
        self.Bamp_1 = Bamp_1

        # compute field of coil 2
        self.B_2 = magpy.getB(self.coil_2, self.grid)
        Bamp_2 = np.linalg.norm(self.B_2, axis=2)
        Bamp_2 /= np.amax(Bamp_2)
        self.Bamp_2 = Bamp_2

        # Combined EM fields of the coils
        self.B = np.add(self.B_1, self.B_2)
        B_amp = np.linalg.norm(self.B, axis=2)
        B_amp /= np.amax(B_amp)
        self.B_amp = B_amp

    # def get_rel2abs_pos(self, rel_pos):
    #     x = self.home[0] + rel_pos[0]
    #     y = self.home[1] + rel_pos[1]
    #     z = self.home[2] + rel_pos[2]
    #     abs_pos = [x, y, z]
    #     return abs_pos

    def get_theta_at_pos(self, uav_pos):
        B_x = self.B[:, :, 0]
        B_y = self.B[:, :, 1]
        theta_grid_xy = np.arctan2(B_y, B_x)
        theta = int(math.degrees(theta_grid_xy[uav_pos[0]][uav_pos[1]]))

        return theta


EM_FIELD = EMField()
