import kalman
import serial_data
import weather_gui
import time

"""
Main python program for reading serial data, computing Kalman estimate for altitude and displaying the user interface
"""

def main():

    kalman_filter = kalman.Kalman(3, 1) # Instantiate a kalman_filter object
    data_packet = serial_data.Data() # Intantiate a serial data object
    gui = weather_gui.WeatherUI()

    while(1):

        if(data_packet.read_serial() == False):
            continue
        else:
            kalman_filter.predict(data_packet.acceleration)
            x, P = kalman_filter.update(data_packet.altitude)

            # TODO: Implement GUI 
            gui.update(data_packet)

if __name__ == '__main__':
    main()