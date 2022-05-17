import time
import datetime
import util
import logging as log


class Transceiver:
    def __init__(self):
        self.max_range = 20  # meters
        self.signal_detected = False  # not detected
        self.direction = -1  # default
        self.distance = -1  # default


transceiver = Transceiver()
timeout_count = 0
timeout = False

uav_pos = [136, 145, 50]  # Example
beacon_pos = [35, 120, 2]  # Example


while True:
    timeout_count += 1

    if timeout_count == 9999:
        timeout = True

    mock_beacon = util.mock_beacon(uav_pos, beacon_pos)
    transceiver.direction = mock_beacon[0]
    transceiver.distance = mock_beacon[1]
    print(f"direction, distance: {transceiver.direction, transceiver.distance}")
    mission_begin_time = datetime.datetime.now()

    if uav_pos[0] == beacon_pos[0] and uav_pos[1] == beacon_pos[1]:
        transceiver.signal_detected = True
        if transceiver.signal_detected:
            current_time = datetime.datetime.now()
            mission_end_time = datetime.datetime.now()
            mission_time = mission_end_time - mission_begin_time
            print(f"\n-------- VICTIM FOUND: {transceiver.signal_detected} -------- ")
            print(f"-- Current ime: {current_time}")
            print("-- Mission time: ",mission_time)
            print(f"-- Location: {uav_pos[0], uav_pos[1], uav_pos[2]}\n")
        break

    if uav_pos[0] < beacon_pos[0]:  # x
        uav_pos[0] += 1
        log.info("x_uav < x_beacon")

    elif uav_pos[0] > beacon_pos[0]:  # x
        uav_pos[0] -= 1
        log.info("x_uav > x_beacon")

    if uav_pos[1] < beacon_pos[1]:  # y
        uav_pos[1] += 1
        log.info("y_uav < y_beacon")

    elif uav_pos[1] > beacon_pos[1]:  # y
        uav_pos[1] -= 1
        log.info("y_uav > y_beacon")

    else:
        print("searching...")
        time.sleep(0.20) # change back
        if timeout:
            print("\n reached timeout \n")
            current_time = datetime.datetime.now()
            mission_end_time = datetime.datetime.now()
            mission_time = mission_end_time - mission_begin_time
            print(f"\n-------- VICTIM NOT FOUND: {transceiver.signal_detected} -------- ")
            print("-- Mission time: ",mission_time)
            break
