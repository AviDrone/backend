from EM_field import EM_field
from util import get_direction
uav_pos = [0,5,5]

test_field = EM_field()
result = test_field.get_theta_at_pos(uav_pos)
direction = get_direction(int(result[0]))
print(int(result[0]))
print(direction)
