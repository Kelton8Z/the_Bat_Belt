import serial
import time
import sys


if __name__ == '__main__':
    # the same baud rate as the one used on Arduino
    ser = serial.Serial('COM3', 9600, timeout=1)
    # clear what could be left in the buffer
    ser.reset_input_buffer()

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
            if line == "SystemOnline":
                break

    time.sleep(1)
    ser.write("activate\n".encode('utf-8'))


    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').rstrip()
                data = line.split(" ")
            #print(data)
                if data[0] == 'Distance:' :
                    dist = int(data[1])
                    print("dist:"+str(dist)+"\n")
                    if dist<30:
                        ser.write("vibrationOn\n".encode('utf-8'))
                    else:
                        ser.write("vibrationOff\n".encode('utf-8'))
                print(line)
        except KeyboardInterrupt:
            ser.write("vibrationOff\n".encode('utf-8'))
            ser.write("deactivate\n".encode('utf-8'))
            print("Bye")
            break

