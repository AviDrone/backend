import EM_field

uav_pos = [128,20,69]

test_field = EM_field.EM_field()
result = test_field.get_theta_at_pos(uav_pos)
print(result)