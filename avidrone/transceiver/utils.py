import numpy as np


def get_displacement(x_1, x_2, y_1, y_2, z_1=0, z_2=0):
    dx = x_2 - x_1
    dy = y_2 - y_1
    dz = z_2 - z_1

    displacement = [dx, dy, dz] # dz = 0 for 2D vector

def get_uclidean_distance(displacement):
    d = (dx ** 2 + dy ** 2 + dz ** 2) ** -2
    dxy = (dx ** 2 + dy ** 2) ** -2
    return dxy

def normalize_vector(self):
    v_d = get_dispslacement(x_1, x_2, y_1, y_2)
    normalized_v_d = v_d / np.linalg.norm(v_d)
    return normalized_v_d

def vector_angle(self):
    fwd = [1,0]
    displacement = get_uclidean_distance(displacement=[1,0,0])
    theta = numpy.acos(numpy.dot(v_d, fwd) / displacement)

def main():
    drone_pos = [1, 2, 0]
    beacon_pos = [2, 4, 0]

    x_1 = drone_pos[0]
    y_1 = drone_pos[1]

    x_2 = beacon_pos[0]
    y_2 = beacon_pos[1]

    displacement = get_displacement(x_1, x_2, y_1, y_2, 0, 0)
    print(displacement)

if __name__ == '__main__':
    main()