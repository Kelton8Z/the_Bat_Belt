import glob
import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import serial.tools.list_ports

def get_ports():

    ports = serial.tools.list_ports.comports()
    
    return ports

def findArduino(portsFound):
    
    commPort = 'None'
    numConnection = len(portsFound)
    
    for i in range(0,numConnection):
        port = foundPorts[i]
        strPort = str(port)
        
        if 'Arduino' in strPort: 
            splitPort = strPort.split(' ')
            commPort = (splitPort[0])

    return commPort
            
                    
foundPorts = get_ports()        
connectPort = findArduino(foundPorts)

print(connectPort)




if __name__ == '__main__':
    # port = glob.glob('/dev/ttyACM*')[0]
    ser = serial.Serial(connectPort, 9600, timeout=1)
    # # clear what could be left in the buffer
    ser.reset_input_buffer()

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().rstrip()
            if line == b'SystemOnline':
                break

    ser.write(b"activateSensor\n")
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().rstrip()
            print(line)
            if line == b'MSG: Sensors have been activated fresh':
                break

    sensor_readings = []

    ser.write(b"reportData\n")

    times = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    l = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0] for i in range(6)]

    fig, axes = plt.subplots(6)
    ln, = plt.plot([],[], 'ro')
    plt.ylim((0,2))




    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').rstrip()
                if line.startswith('data'):
                    _, *sensor_readings = line.split(' ')
                    assert(len(sensor_readings)==6)
    #                     module_indices.append(module_idx)
                        #historical_readings[module_idx].append(sensor_reading)
                elif line.startswith('msg'):
                    print(line)
                elif line.startswith('error'):
                    print(line)
            print(sensor_readings)


            for i in range(6):
                y = 0
                if i < len(sensor_readings):
                    y = sensor_readings[i]
                l[i].append(y)
                l[i].pop(0)
                axes[i].clear()
                axes[i].plot(times, l[i])
                plt.pause(0.01)



        except KeyboardInterrupt:
            ser.write(b"deactivateSensor\n")
            print("program stopped\n")
            break
        
    
    plt.show()