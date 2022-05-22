import datetime
import logging
import time

import util

# log
log = logging.getLogger(__name__)
log.setLevel(logging.INFO)

formatter = logging.Formatter('%(asctime)s:%(levelname)s:%(message)s')

file_handler = logging.FileHandler('transceiver.log')
file_handler.setFormatter(formatter)

log.addHandler(file_handler)

class Transceiver:
    def __init__(self):
        self.direction = -1  # not detected
        self.distance = -1  # not detected
        self.signal_detected = False  # not detected
        self.position = [None, None, None]  # not detected

        # settings
        self.mode = "transmit"  # or detect
        self.model_number = 0  # Avidrone (default)
        
        
        self.battery = (
            720000 if self.mode == "transmit" else 180000   # seconds in battery at 100%
        )   # Battery life from: https://www.backcountry.com/backcountry-access-tracker-dts-beacon#

        self.model = [
            "Avidrone",
            "Tracker DTS",
            "Orthovox Zoom+",
            "Orthovox 3+",
            "Tracker2",
            "Tracker3",
            "Pieps DSP Sport",
            "Pieps DSP Pro",
            "Mammut Pulse",
            "Orthovox S1",
        ]   # Model reference: https://beaconreviews.com/search_strip_widths.php

        self.search_strip_width = [
            {"Avidrone": 6},
            {"Tracker DTS": 34},
            {"Orthovox Zoom+": 31},
            {"Orthovox 3+": 36},
            {"Tracker2": 38},
            {"Tracker3": 39},
            {"Pieps DSP Sport": 41},
            {"Pieps DSP Pro": 42},
            {"Mammut Pulse": 54},
            {"Orthovox S1": 50},
        ]

    
    def get_model(self, model_number):
        return transceiver.model[model_number]
        
    def switch_mode(self):
        if self.mode == "transmit":
            self.mode = "receive"
        elif self.mode == "receive":
            self.mode = "transmit"
        else:
            log.warning("Unknown mode")

    def victim_found_msg(self):
        print(f"\n-------- VICTIM FOUND: {transceiver.signal_detected} -------- ")
        print(f"-- Current ime: {current_time}")
        print("-- Mission time: ", mission_time)
        print(f"-- Location: {transceiver.position}\n")
        
    def victim_not_found_msg(self):
        print(f"\n-------- VICTIM FOUND: {transceiver.signal_detected} -------- ")
        print(f"-- Current ime: {current_time}")
        print(f"-- Mission time: {mission_time}")

        
    @staticmethod
    def show_data():
        print("\n--- transceiver data ---")
        print(f"transceiver.mode: {transceiver.mode}")
        print(f"transceiver.direction: {transceiver.direction}")
        print(f"transceiver.distance: {transceiver.distance}")
        print(f"transceiver.signal_detected: {transceiver.signal_detected}")
    
    @staticmethod
    def show_settings():
        print("\n--- settings ---")
        print(f"transceiver.mode: {transceiver.mode}")
        print(f"transceiver.model: {model}")
        print(
            "transceiver.search_strip_width: ",
            transceiver.search_strip_width[transceiver.model_number][model],
        )


IS_TEST = True  # set to true for simulation

transceiver = Transceiver()
transceiver.model_number = 0  # Avidrone
model = transceiver.get_model(transceiver.model_number)

timeout_count = 0
timeout = False

uav_pos = [36, 145, 50]  # Example
beacon_pos = [31, 146, 51]  # Example
# beacon_pos = [35, 120, 2]  # Example

while True:
    timeout_count += 1
    if timeout_count == transceiver.battery: timeout = True

    if IS_TEST:
        mock_transceiver = util.mock_transceiver(uav_pos, beacon_pos)
        transceiver.direction = mock_transceiver[0]
        transceiver.distance = mock_transceiver[1]
    
    else:
        transceiver.direction = -1  # TODO real vals
        transceiver.distance = -1   # TODO real vals
    
    print(f"-- direction, distance: {(transceiver.direction, transceiver.distance)}")
    mission_begin_time = datetime.datetime.now()

    if uav_pos[0] == beacon_pos[0] and uav_pos[1] == beacon_pos[1]:
        transceiver.signal_detected = True
        
        if transceiver.signal_detected:
            current_time = datetime.datetime.now()
            mission_end_time = datetime.datetime.now()
            mission_time = mission_end_time - mission_begin_time
            transceiver.position = uav_pos
            transceiver.victim_found_msg()
            transceiver.show_settings()
        break

    
    if uav_pos[0] < beacon_pos[0]:  # x
        uav_pos[0] += 1
        # log.debug("x_uav < x_beacon")

    elif uav_pos[0] > beacon_pos[0]:  # x
        uav_pos[0] -= 1
        # log.debug("x_uav > x_beacon")

    if uav_pos[1] < beacon_pos[1]:  # y
        uav_pos[1] += 1
        # log.debug("y_uav < y_beacon")

    elif uav_pos[1] > beacon_pos[1]:  # y
        uav_pos[1] -= 1
        # log.debug("y_uav > y_beacon")

    else:
        # time.sleep(0.5)  # Transceiver receives reading every half seconds
        if timeout:
            log.warning("\n reached timeout \n")
            current_time = datetime.datetime.now()
            mission_end_time = datetime.datetime.now()
            mission_time = mission_end_time - mission_begin_time
            transceiver.victim_not_found_msg()
            break
