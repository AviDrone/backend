import util


class Transceiver:
    def __init__(self, uav_pos, beacon_pos):
        position = [-1, -1]
        self.min_range = 5
        self.max_range = 27
        self.signal_status = False
        self.beacon_pos = [0, 0, 0]  # default, transceiver signal not acquired
        self.mock = util.mock_beacon(uav_pos, beacon_pos)  # Mock transceiver object
