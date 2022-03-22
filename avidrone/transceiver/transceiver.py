class Transceiver:
    def __init__(self, position, signal_status):
        position = [-1, -1]
        self.min_range = 5
        self.max_range = 27
        self.signal_status = signal_status
        self.position = position # default, transceiver signal not acquired

    @staticmethod
    def signal_status(self, signal_status):
        signal_acquired = False
        signal_detected = signal_status

        if signal_detected:
            signal_acquired = True
        return signal_acquired

    @staticmethod
    def bin_direction(self, position, min_range, max_range):
        """
        0 = forward
        1 = left
        2 = right
        """
        min_val = min_range
        max_val = max_range
        
        new_direction = -1  # default, error
        direction = position[0]
        distance = position[1]

        if direction == 2:
            new_direction = 0
        elif direction < 2 and distance < min_val:
            new_direction = 1
        elif direction > 2 and distance < max_val:
            new_direction = 2

        return new_direction
