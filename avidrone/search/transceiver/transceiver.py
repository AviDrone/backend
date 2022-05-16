from time import sleep
import datetime
import util


class Transceiver:
    def __init__(self):
        self.max_range = 20  # meters
        self.signal_detected = False  # not detected
        self.direction = -1     # default
        self.distance = -1   # default


transceiver = Transceiver()
timeout_count = 0
timeout = False
# --- scratch ----
uav_pos = [35.363262, 149.165237, 25.0]  # Example
beacon_pos = [35.363862, 149.165337, 0.0]  # Example


while True:
    timeout_count += 1
    
    if timeout_count == 999:
        timeout = True

    mock_beacon = util.mock_beacon(uav_pos, beacon_pos)
    transceiver.direction = mock_beacon[0]
    transceiver.distance = mock_beacon[1]
    print(f"direction, distance: {transceiver.direction, transceiver.distance}")

    if uav_pos[0] == beacon_pos[0] and uav_pos[1] == beacon_pos[1]:
        transceiver.signal_detected = True
        if transceiver.signal_detected:
            current_time = datetime.datetime.now()
            print(f"\n-------- VICTIM FOUND: {transceiver.signal_detected} -------- ")
            print(f"-- Time: {current_time}")
            print(f"-- Location: {beacon_pos[0], beacon_pos[1], beacon_pos[2]}\n")
        break
    
    if uav_pos[0] < beacon_pos[0]:  # x
        uav_pos[0] += 0.000001
        print("x_uav < x_beacon")

    elif uav_pos[0] > beacon_pos[0]:  # x
        uav_pos[0] -= 0.000001
        print("x_uav > x_beacon")
        
    if uav_pos[1] < beacon_pos[1]:  # y
        uav_pos[1] += 0.000001
        print("y_uav < y_beacon")

    elif uav_pos[1] > beacon_pos[1]:  # y
        uav_pos[1] -= 0.000001
        print("y_uav > y_beacon")

    else:
        print("searching...")        
        if timeout:
            print("\n reached timeout \n")
            current_time = datetime.datetime.now()
            print(f"\n-------- VICTIM FOUND: {transceiver.signal_detected} -------- ")
            print(f"-- Time: {current_time}")
            break
