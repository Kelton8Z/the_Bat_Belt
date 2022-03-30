#!/usr/bin/env python3
import serial
import cv2
import numpy as np
from oak_d_test import getDepthFrame
# import oak_d_test
from collections import defaultdict
from sklearn.linear_model import LinearRegression

if __name__ == '__main__':
    imgFrame = getDepthFrame()
    frame = imgFrame.getFrame()
    # calibration 
    timestamp = imgFrame.getTimestamp()
    print(timestamp)
    frame = frame[:, 33:-33] # cut nonoverlapping area of binocular 
    # 400 x 574 corresponds to 65°, where valid values are within [350, 5520], ignore 0s
    _, colNum = frame.shape
    leftFrame, rightFrame = frame[:colNum//2], frame[:colNum//2:]
    baseline = frame[colNum//2]
    model = LinearRegression()
    x = np.array([i for i in range(len(baseline))]).reshape(-1, 1)
    line = model.fit(x, baseline)
    pred = line.predict(x)
    print(pred)
    # the same baud rate as the one used on Arduino
    # ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    # # clear what could be left in the buffer
    # ser.reset_input_buffer()
    # module_indices = []
    # historical_readings = defaultdict(list)
    # # distance_threshold = 100
    # for i in range(1, 7):
    #     historical_readings[i] = [0]

    # while True:
    #     if ser.in_waiting > 0:
    #         line = ser.readline().decode('utf-8').rstrip()
    #         if line == 'SystemOnline':
    #             break
    
    # ser.write(b"activateSensor")
    # while True:
    #     if ser.in_waiting > 0:
    #         line = ser.readline().decode('utf-8').rstrip()
    #         # data: <sensornum 1-6> <reading 0-500> 
    #         # msg: <debug msg>
    #         print(line)
    #         if line.startswith('data'):
    #             _, module_idx, sensor_reading = line.split(' ')
    #             module_idx = int(module_idx)
    #             sensor_reading = int(sensor_reading)
    #             module_indices.append(module_idx)
    #             historical_readings[module_idx].append(sensor_reading)
    #         elif line.startswith('msg'):
    #             pass
    #         elif line.startswith('error'):
    #             print(line)
        
            
    #     # Rate “threat level” of each identified obstacle based on distance and speed
    #     # Activate vibration system accordingly
    #     # intensities = [0 for i in range(6)]
        
    #     # modifyVibrator <num 1-6><level 1-3>
    #     # deactivateVibrator <num>
    #     # deactivateSensor
    #     for module_idx in module_indices:
    #         if len(historical_readings[module_idx]) >= 1:
    #             print(historical_readings)
    #             current_reading = historical_readings[module_idx][-1]
            
    #             last_reading = historical_readings[module_idx].pop(0)
    #             dx = current_reading - last_reading
    #             # 200ms between readings
    #             # e.g. 1m/s = 200cm/0.2s
    #             if dx > 300 or current_reading <= 100: # speed>3m/s or distance <= 1m
    #                 intensity = 3
    #             elif dx > 200 or current_reading <= 200: # speed>2m/s or distance <= 2m
    #                 intensity = 2
    #             elif dx > 100 or current_reading <= 450: # speed>1m/s or distance <= 4.5m
    #                 intensity = 1
    #             else:
    #                 intensity = 0
    #             print(intensity)
    #             if intensity > 0 and intensity != last_intensities[module_idx]:
    #                 last_intensities[module_idx] = intensity
    #                 ser.write(bytes("modifyVibrator " + str(module_idx) + str(intensity), 'utf-8'))
    #                 print("modifyVibrator " + str(module_idx) +  str(intensity))
    #             else:
    #                 ser.write(bytes("modifyVibrator " + str(module_idx), 'utf-8'))

    #     # if input("break ?")=='b':
    #     # if keyboard.read_key():
    #     #     for module_idx in range(1, 7):
    #     #         ser.write(bytes("modifyVibrator " + str(module_idx) +  '0', 'utf-8'))
    #     #         ser.write(bytes("deactivateSensor", 'utf-8'))
    #     #     break