class Transceiver:
    def __init__(self):
        position = [-1, -1]
        self.min_range = 5
        self.max_range = 27
        self.signal_status = False
        self.position = [0, 0, 0]  # default, transceiver signal not acquired
