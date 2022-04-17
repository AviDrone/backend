import util
# For interfacing with the C code
import ctypes
import pathlib

class Transceiver:
    def __init__(self, uav_pos, beacon_pos, direction, distance):

        self.direction = direction
        self.distance = distance

        self.min_range = 5
        self.max_range = 27
        self.signal_status = False
        self.uav_pos = [-1,-1,-1]   # TODO change. Default
        self.beacon_pos = [0, 0, 0]  # TODO change. Default, transceiver signal not acquired
        self.mock = asList(util.mock_beacon(uav_pos, beacon_pos))  # Mock transceiver object
        self.read = [self.direction, self.distance] # Real transceiver object


    def read_transceiver(self):
        shared_ctypes_lib = pathlib.Path().absolute() / "read_transceiver.lib"
        c_lib = ctypes.CDLL(shared_ctypes_lib)

        output_array = (ctypes.c_double * 2)()
        print("output_array before:", output_array[0], " ", output_array[1])
        c_lib._Z11get_dir_digPd(ctypes.byref(output_array))
        print("Direction: ", output_array[0])
        print("Distance: ",output_array[1])

        return Transceiver(output_array[0], output_array[1])
