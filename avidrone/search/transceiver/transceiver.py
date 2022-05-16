from time import sleep
import util


class Transceiver:
    def __init__(self):
        self.max_range = 20  # meters
        self.signal_detected = False  # not detected
        self.direction = -1     # default
        self.distance = -1   # default


transceiver = Transceiver()

# --- scratch ----
uav_pos = [1200, 120, 25]  # Example
beacon_pos = [60, 60, 1]  # Example

print("direction:", transceiver.direction)
print("distance: ", transceiver.distance)

while True:
    mock_beacon = util.mock_beacon(uav_pos, beacon_pos)
    transceiver.direction = mock_beacon[0]
    transceiver.distance = mock_beacon[1]
    print("direction:", transceiver.direction)
    print("distance: ", transceiver.distance)

    if uav_pos[0] < beacon_pos[0]:
        uav_pos[0] += 1
        print("getting closer")

    elif uav_pos[0] > beacon_pos[0]:
        uav_pos[0] -= 1
        print("getting closer_")

    elif uav_pos[0] == beacon_pos[0]:
        transceiver.signal_detected = True
        print("Victim found: ", transceiver.signal_detected)
        break

    else:
        print("Victim not found")
