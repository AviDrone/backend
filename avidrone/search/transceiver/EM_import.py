from EM_field import EM_field
from util import get_direction

home = [46.0452822, -118.3930353, 584]
rel_pos = [40, 20, 24]

test_field = EM_field()
result = test_field.get_theta_at_pos(rel_pos)
direction = get_direction(int(result))


def print_result(result, direction):
    print(f"flux direction, output direction: {int(result)}, {direction}")


def rel2abs(home, rel_pos):
    x = home[0] + rel_pos[0]
    y = home[1] + rel_pos[1]
    z = home[2] + rel_pos[2]

    abs_pos = [x, y, z]
    return abs_pos


test = rel2abs(home, rel_pos)
print_result(result, direction)
print(test)
