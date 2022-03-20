class Transceiver:
    def __init__(self, position, signal_status, direction,distance):
        self.position = position
        self.signal_status = signal_status
        self.direction = direction
        self.distance = distance

    def set_position(self, x, y, z):
        position = [x,y,z]  # TODO implement
        return position
    

    def get_position(self, x, y, z):
        position = [x,y,z]  # TODO implement
        return position

    def signal_status(self, signal_status):
        signal_detected = signal_status
        signal_acquired = False
        if signal_detected: signal_acquired = True
        return signal_acquired
