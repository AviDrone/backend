import util


class Transceiver:
    def __init__(self):
        self.max_range = 20  # meters
        self.direction = -1  # not detected
        self.distance = -1  # not detected
        self.signal_detected = False  # not detected

transceiver = Transceiver()

# --- scratch ----
uav_pos = [120, 120, 25]  # Example
beacon_pos = [60, 60, 1]  # Example

while True:
    mock_beacon = util.mock_beacon(uav_pos, beacon_pos)
    print("Direction, Distance: ", mock_beacon)
    
    if uav_pos[0] < beacon_pos[0]:
        uav_pos[0] += 1
        
    elif uav_pos[0] > beacon_pos[0]:
        uav_pos[0] -= 1
        
    elif uav_pos[0] == beacon_pos[0]:
        break
    
    else:
        print("test")
    