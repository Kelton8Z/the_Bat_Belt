#!/usr/bin/env python3
import serial

if __name__ == '__main__':
    # the same baud rate as the one used on Arduino
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    # clear what could be left in the buffer
    ser.reset_input_buffer()

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
