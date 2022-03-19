#!/usr/bin/env python3
import serial
import cv2
# from oak_d_test import device, frame, depth
import oak_d_test
from collections import defaultdict

if __name__ == '__main__':
    # the same baud rate as the one used on Arduino
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    # clear what could be left in the buffer
    ser.reset_input_buffer()
    historical_readings = defaultdict(list)
    # distance_threshold = 100
    for i in range(1, 7):
        historical_readings[i] = [0]

    while True:
        ser.write("activateSensor")
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            # data: <sensornum 1-6> <reading 0-500> 
            # msg: <debug msg>
            print(line)
            if line.startswith('data'):
                _, module_idx, sensor_reading = line.split(' ')
                historical_readings[module_idx].append(sensor_reading)
            elif line.startswith('msg'):
                pass

        edgeLeftFrame, edgeRightFrame, edgeRgbFrame = oak_d_test.getEdgeFrames()
        # Show the frame
        cv2.imshow(oak_d_test.edgeLeftStr, edgeLeftFrame)
        cv2.imshow(oak_d_test.edgeRightStr, edgeRightFrame)
        cv2.imshow(oak_d_test.edgeRgbStr, edgeRgbFrame)

        # frame = oak_d_test.getDisparityFrame()

        # cv2.imshow("disparity", frame)

        # Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
        # frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
        # cv2.imshow("disparity_color", frame)
       
        # if frame is not None:
        #     for detection in detections:
        #         # for each bounding box, we first normalize it to match the frame size
        #         bbox = frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
        #         # and then draw a rectangle on the frame to show the actual result
        #         cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (255, 0, 0), 2)
        #     # After all the drawing is finished, we show the frame on the screen
        #     cv2.imshow("preview", frame)

        # at any time, you can press "q" and exit the main loop, therefore exiting the program itself
        if cv2.waitKey(1) == ord('q'):
            break
            
        # Rate “threat level” of each identified obstacle based on distance and speed
        # Activate vibration system accordingly
        # intensities = [0 for i in range(6)]
        
        # activateVibrator <num 1-6> <level 1-3>
        # deactivateVibrator <num>
        # deactivateSensor
        
        for module_idx in module_indices:
            intensity = intensities[module_idx-1]
            last_reading = historical_readings[module_idx].pop(0)
            dx = historical_readings[module_idx][-1] - last_reading
            if dx > 300:
                intensity = 3
            elif dx > 200:
                intensity = 2
            elif dx > 100:
                intensity = 1
            if intensity > 0:
                ser.write("activateVibrator " + module_idx +  intensity)
            else:
                ser.write("deactivateVibrator " + module_idx)
