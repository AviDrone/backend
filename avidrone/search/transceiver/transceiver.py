import datetime
import logging as log
import time

import util


class Transceiver:
    """
    # For more info about avalanche beacons:
    # https://avalanche.org/avalanche-encyclopedia/avalanche-transceiver-beacon/

    """

    def __init__(self):
        # sensor outputs
        self.direction = -1  # not detected
        self.distance = -1  # not detected
        self.position = [-1, -1, -1]  # not detected
        self.signal_detected = False  # not detected

        # settings
        self.mode = 'transmit'
        self.model_number = 0  # default: Avidrone

        # Only models with measured range were selected
        # Model reference: https://beaconreviews.com/search_strip_widths.php
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
        ]

        self.search_strip_width = [
            {'Avidrone': 6},
            {'Tracker DTS': 34},
            {'Orthovox Zoom+': 31},
            {'Orthovox 3+': 36},
            {'Tracker2': 38},
            {'Tracker3': 39},
            {'Pieps DSP Sport': 41},
            {'Pieps DSP Pro': 42},
            {'Mammut Pulse': 54},
            {'Orthovox S1': 50},
        ]

        # simulation parameters
        self.frequency = 457  # kHz

        # Battery life from: https://www.backcountry.com/backcountry-access-tracker-dts-beacon#
        self.battery = 720000 if self.mode == 'transmit' else 180000  # seconds in battery at 100%

    def switch_mode(self):
        if self.mode == "transmit":
            self.mode = "receive"
        elif self.mode == "receive":
            self.mode = "transmit"
        else:
            log.error("Unknown mode")

    @staticmethod
    def show_settings():
        print("------ TRANSCEIVER ------")
        print("--- data ---")
        print(f"transceiver.direction: {transceiver.direction}")
        print(f"transceiver.distance: {transceiver.distance}")
        print(f"transceiver.signal_detected: {transceiver.signal_detected}")
        print("\n--- settings ---")
        print(f"transceiver.mode: {transceiver.mode}")
        print(f"transceiver.model: {model}")
        print("transceiver.search_strip_width: ", transceiver.search_strip_width[transceiver.model_number][model])
        print("\n--- simulation parameters ---")
        print(f"transceiver.frequency: {transceiver.frequency}")


transceiver = Transceiver()
transceiver.model_number = 0  # Avidrone
model = transceiver.model[transceiver.model_number]

timeout_count = 0
timeout = False

uav_pos = [136, 145, 50]  # Example
beacon_pos = [35, 120, 2]  # Example

transceiver.show_settings()

while True:
    timeout_count += 1

    if timeout_count == transceiver.battery:
        timeout = True

    mock_beacon = util.mock_beacon(uav_pos, beacon_pos)
    transceiver.direction = mock_beacon[0]
    transceiver.distance = mock_beacon[1]
    log.info("direction, distance: ", (transceiver.direction, transceiver.distance))
    mission_begin_time = datetime.datetime.now()

    if uav_pos[0] == beacon_pos[0] and uav_pos[1] == beacon_pos[1]:
        transceiver.signal_detected = True
        if transceiver.signal_detected:
            current_time = datetime.datetime.now()
            mission_end_time = datetime.datetime.now()
            mission_time = mission_end_time - mission_begin_time
            print(f"\n-------- VICTIM FOUND: {transceiver.signal_detected} -------- ")
            print(f"-- Current ime: {current_time}")
            print("-- Mission time: ", mission_time)
            transceiver.position = uav_pos
            print(f"-- Location: {transceiver.position}\n")
            
            transceiver.show_settings()

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
        # time.sleep(0.5)  # Transceiver receives reading every half seconds
        if timeout:
            print("\n reached timeout \n")
            current_time = datetime.datetime.now()
            mission_end_time = datetime.datetime.now()
            mission_time = mission_end_time - mission_begin_time
            print(
                f"\n-------- VICTIM NOT FOUND: {transceiver.signal_detected} -------- "
            )
            print("-- Mission time: ", mission_time)
            break
