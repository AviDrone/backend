class Transceiver:
    def __init__(
            self,
            position,
            signal_status,
            direction,
            distance
    ):
        position = self.position
        signal_status = self.signal
        self.direction = direction
        self.distance = distance

    def set_position(self, x, y, z):
        pass

    def get_position(self, x, y, z):
        pass


class Signal(Transceiver):
    def __init__(
            self,
            position,
            signal_status,
            direction,
            distance
        ):
        super().__init__(position, signal_status, direction, distance)

    def signal_status(self, status):
        signal_detected = False
        signal_acquired = False

        if signal_detected: signal_acquired = True
        return signal_acquired
