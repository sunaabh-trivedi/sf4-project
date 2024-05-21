"""
Reading, validating and storing the most recent sensor data received over the serial port
"""

import serial
import struct
import zlib

def validate_checksum(data, checksum):

    received_crc32 = checksum
    computed_crc32 = zlib.crc32(data)

    if(received_crc32 != computed_crc32):
        return False

    return True

class Data():

    def __init__(self):

        self.packet_size = 26

        self.temperature = 0.0
        self.humidity = 0.0
        self.wind_speed = 0.0
        self.wind_direction = 0.0
        self.altitude = 0.0
        self.acceleration = 0.0
        self.climb_rate = 0.0

        self.ser = serial.Serial(baudrate=38400, timeout=0.01)
    
    def read_serial(self):
        
        raw_bytes= self.ser.read(self.packet_size)
        num_bytes = len(raw_bytes)

        # If all the data was not captured due to timeout, don't update data
        if(num_bytes != self.packet_size):
            print("Timeout occurred")
            return False

        view = memoryview(raw_bytes)

        checksum = view[21:25]

        # If the checksum is invalid, don't update data
        if(validate_checksum(view[0:21], checksum) == False):
            print("Invalid checksum")
            return False

        # Update data
        self.altitude = struct.unpack('f', view[0:4])
        self.acceleration = struct.unpack('f', view[4:8])
        self.temperature = struct.unpack('f', view[8:12])
        self.humidity = struct.unpack('f', view[12:16])
        self.wind_speed = struct.unpack('f', view[16:20])
        self.wind_direction = struct.unpack('f', view[20])

        return True







