#!/usr/bin/env python3
import serial
import cv2
from oak_d_test import device, q_rgb, q_nn, frame, detections, frameNorm

if __name__ == '__main__':
    # the same baud rate as the one used on Arduino
    # ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    # clear what could be left in the buffer
    # ser.reset_input_buffer()

    while True:
        # if ser.in_waiting > 0:
        #     line = ser.readline().decode('utf-8').rstrip()
        #     print(line)

       # we try to fetch the data from nn/rgb queues. tryGet will return either the data packet or None if there isn't any
        in_rgb = q_rgb.tryGet()
        in_nn = q_nn.tryGet()

        if in_rgb is not None:
            # If the packet from RGB camera is present, we're retrieving the frame in OpenCV format using getCvFrame
            frame = in_rgb.getCvFrame()

        if in_nn is not None:
            # when data from nn is received, we take the detections array that contains mobilenet-ssd results
            detections = in_nn.detections

        if frame is not None:
            for detection in detections:
                # for each bounding box, we first normalize it to match the frame size
                bbox = frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
                # and then draw a rectangle on the frame to show the actual result
                cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (255, 0, 0), 2)
            # After all the drawing is finished, we show the frame on the screen
            cv2.imshow("preview", frame)

        # at any time, you can press "q" and exit the main loop, therefore exiting the program itself
        if cv2.waitKey(1) == ord('q'):
            break
            
        # Rate “threat level” of each identified obstacle based on distance and speed
        # Activate vibration system accordingly
        intensity = -1
        # 
     

