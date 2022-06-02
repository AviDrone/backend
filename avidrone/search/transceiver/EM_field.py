from re import S

import magpylib as magpy
import matplotlib.pyplot as plt
import numpy as np

"""
# Simulation of electromagnetic field of transceiver coils
# For more information about the coils, see: 
# https://magpylib.readthedocs.io/en/latest/examples/examples_30_coil_field_lines.html
"""


class EM_field:
    def __init__(self):
        # create grid
        self.ts = np.linspace(-100, 100, 100)
        self.grid = np.array([[(x, 0, z) for x in self.ts] for z in self.ts])

        # coil 1
        ts = np.linspace(-6, 6, 500)
        vertices_1 = np.c_[5 * np.cos(ts * 2 * np.pi), 5 * np.sin(ts * 2 * np.pi), ts]
        coil_1 = magpy.current.Line(current=7500, vertices=vertices_1)  # AAA battery
        self.coil_1 = coil_1.rotate_from_angax(45, "y")  # Front coil

        # coil 2
        ts = np.linspace(-6, 6, 500)
        vertices_2 = np.c_[5 * np.cos(ts * 2 * np.pi), 5 * np.sin(ts * 2 * np.pi), ts]
        coil_2 = magpy.current.Line(current=7500, vertices=vertices_2)  # AAA battery
        self.coil_2 = coil_2.rotate_from_angax(-45, "y")  # Back coil

        # compute field of coil 1
        self.B_1 = magpy.getB(self.coil_1,self. grid)
        Bamp_1 = np.linalg.norm(self.B_1, axis=2)
        Bamp_1 /= np.amax(Bamp_1)
        self.Bamp_1 = Bamp_1

        # compute field of coil 2
        self.B_2 = magpy.getB(self.coil_2, self.grid)
        Bamp_2 = np.linalg.norm(self.B_2, axis=2)
        Bamp_2 /= np.amax(Bamp_2)
        self.Bamp_2 = Bamp_2

        self.B = np.add(self.B_1, self.B_2)
        B_amp = np.linalg.norm(self.B, axis=2)
        B_amp /= np.amax(B_amp)
        self.B_amp  = B_amp # Combined EM fields of coil 1 and coil 2
                
        
    def get_theta_at_pos(self, uav_pos):
        self.B
        
        uav_pos_x = uav_pos[0]
        uav_pos_y = uav_pos[1]
        
        B_x = self.B[:, :, 0]
        B_y = self.B[:, :, 1]
        # B_z = B[:, :, 2]  # currently not used for 2D model

        theta_grid = []  # empty list
        theta_val = np.arctan2(B_y, B_x)

        for i in range(len(theta_val)):
            for j in range(len(theta_val[i])):
                theta_grid.append(theta_val[i][j])

        return theta_grid[uav_pos_x][uav_pos_y]