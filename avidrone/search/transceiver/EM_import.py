from dataclasses import astuple
from EM_field import EM_field
from util import get_direction
import utm

home = [46.0452822, -118.3930353, 584]  # test: where the beacon is
rel_pos = [40, 20, 24]  # test: where the uav is

# Initialization
test_field = EM_field()
theta = test_field.get_theta_at_pos(rel_pos)
direction = get_direction(theta)

latlon2utm = utm.from_latlon(home[0], home[1])
utm_pos = []
for var in latlon2utm:
    utm_pos.append(var)
    print(utm_pos[latlon2utm.index(var)])

def add_rel_pos(utm_pos, rel_pos):
    new_ = []    
    new_.append(utm_pos[0] + rel_pos[0])
    new_.append(utm_pos[1] + rel_pos[1])
    new_.append(utm_pos[2])
    new_.append(utm_pos[3])
    return new_

new_utm = add_rel_pos(utm_pos, rel_pos)
utm2latlon = utm.to_latlon(new_utm[0], new_utm[1], new_utm[2], new_utm[3])
print(utm2latlon)



def print_result(theta, direction):
    print(f"flux direction, output direction: {int(theta)}, {direction}")


# test = EM_field.get_rel2abs_pos(test_field, rel_pos)
# print_result(theta, direction)
# print(test)
