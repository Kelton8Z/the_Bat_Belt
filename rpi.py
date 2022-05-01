#!/usr/bin/env python3
import glob
import serial
import cv2
import numpy as np
from oak_d_test import getAugmentedFeature
# import oak_d_test
from collections import defaultdict
from sklearn.linear_model import Ridge

import matplotlib.pyplot as plt

import numpy as np
import pyttsx3
FRAME_THRESHOLD = 640

james = pyttsx3.init()
def james_speak(content):
    james.say(content)
    james.runAndWait()
    print(content)



def getDepthThreshold():
    BL = 75 #mm
    HFOV = 72 #deg
    Dv = (BL / 2) * np.tan((90 - HFOV / 2) * np.pi / 180)
    W = 1280 #720p
    D = np.empty(W)
    numerator = W * Dv
    for B in range(W // 2):
        D[B] = numerator / (B+0.5)
        D[-(B + 1)] = D[B]
    return D

def filterInvalidDepth(stereoFrame, D):
    row, col = stereoFrame.shape
    for i in range(row):
        for j in range(col):
            if stereoFrame[i][j] < D[j]:
                stereoFrame[i][j] = 0
    return stereoFrame

def getRowBase(midBase, col, angleStep, midCol = 640, HFOV = 71.9):
    colDiff = np.abs(col - midCol)
    # return midBase / np.cos(colDiff * angleStep)
    return midBase

def vecToDict(vec):
    d = {}
    rowNum = vec.shape[0]
    print(rowNum)
    assert(rowNum==720)
    firstBoundary = rowNum // 3
    assert(firstBoundary==240)
    secondBoundary = 2*(rowNum//3)

    d['mid'] = vec[firstBoundary:secondBoundary]
    d['top'] = vec[:firstBoundary]
    d['bottom'] = vec[secondBoundary:]
    return d

def frameToDict(frame):
    ground_level_readings = {}

    rowNum, colNum = frame.shape
    originalLeft = frame[:, :colNum//2]
    originalRight = frame[:, colNum//2:]
    rightVecDic = vecToDict(originalRight)
    leftVecDic = vecToDict(originalLeft)
    ground_level_readings["left"] = dict({"original" : originalLeft})
    ground_level_readings["left"].update(leftVecDic)
    ground_level_readings["right"] = dict({"original" : originalRight})
    ground_level_readings["right"].update(rightVecDic)
    return ground_level_readings


def normalize(vals):
    res = (np.mean(vals) - min(vals)) / (max(vals) - min(vals))
    assert(res>=0)
    assert(res<1)
    return res


def rateSubframe(ground_level, baseMat):
    '''
        only the distance and not the speed matters for ground level obstacles
        return alert levels of 0/1/2/3
    '''
    # vecSize = baseMatDict['top'].shape[0]
    # assert(vecSize==400)
    #assert(ground_level_dict['top'].shape==(240, 427))

    #top_diff = normalize(np.square(np.subtract(np.mean(ground_level_dict['top'], axis=1).T, baselineDict['top'].T)))
    #mid_diff = normalize(np.square(np.subtract(np.mean(ground_level_dict['mid'], axis=1).T, baselineDict['mid'].T)))
    #bottom_diff = normalize(np.square(np.subtract(np.mean(ground_level_dict['bottom'], axis=1).T, baselineDict['bottom'].T)))
    rowNum = ground_level.shape[0]
    # print(rowNum)
    assert(rowNum==720)
    '''
    firstBoundary = rowNum // 3
    assert(firstBoundary==240)
    secondBoundary = 2*firstBoundary

    top_diffs = []
    mid_diffs = []
    bottom_diffs = []

    for feature in features:
        x, y = int(feature.position.x), int(feature.position.y)
        feature_IDs.add(feature.id)
        if x < 420:
            error = (stereoFrame[x][y] - baseMat[x][y])**2
            if y < firstBoundary:
                top_diffs.append(error)
            elif y > secondBoundary:
                bottom_diffs.append(error)
            else:
                mid_diffs.append(error)

    top_diff = np.mean(np.array(top_diffs))
    mid_diff = np.mean(np.array(mid_diffs))
    bottom_diff = np.mean(np.array(bottom_diffs))


    top_score = 0
    mid_score = 0
    bottom_score = 0
    if top_diffs:
        top_score = 1/(1+np.exp(-top_diff))
    if mid_diffs:
        mid_score = 1/(1+np.exp(-mid_diff))
    if bottom_diffs:
        bottom_score = 1/(1+np.exp(-bottom_diff))

    level = (top_score + 2*mid_score + 3*bottom_score)//6
    print(top_diff)
    print(mid_diff)
    print(bottom_diff)'''
    #print(level)
    divided = ground_level / baseMat
    height_threshold = 0.95
    height_frame = np.zeros_like(divided)
    height_frame[divided > height_threshold] = 1

    num_of_pixel = 50000
    zero_num = np.sum(np.sign(-ground_level + 1))
    zero_frame = np.sign(-ground_level) + 1
    lv1_threshold = 1000
    lv2_threshold = 500
    lv2_frame = np.zeros_like(ground_level)
    lv2_frame[ground_level < lv2_threshold] = 1
    lv1_frame = np.zeros_like(ground_level)
    lv1_frame[ground_level < lv1_threshold] = 1
    sum_frame = lv1_frame + lv2_frame
    sum_frame[zero_frame == 1] = 0
    filtered_frame = np.copy(sum_frame)
    filtered_frame[height_frame == 1] = 0
    """
    plt.figure()
    _, axarr = plt.subplots(2, 4)
    axarr[0,0].imshow(disparityFrame)
    axarr[0,0].set_title("disparity")
    axarr[0,1].imshow(zero_frame)
    axarr[0,2].imshow(lv3_frame)
    axarr[0,3].imshow(lv2_frame)
    axarr[1,0].imshow(lv1_frame)
    axarr[1,1].imshow(sum_frame)
    axarr[1,1].set_title("threat level (no filter)")
    axarr[1,2].imshow(height_frame)
    axarr[1,3].imshow(filtered_frame)
    axarr[1,3].set_title("threat level (ground filtered)")
    #plt.show()
    #sys.exit()
    """
    if np.sum(filtered_frame[filtered_frame >= 3].astype(int)) > num_of_pixel:
        return 3
    if np.sum(filtered_frame[filtered_frame >= 2].astype(int)) > num_of_pixel:
        return 2
    if np.sum(filtered_frame[filtered_frame >= 1].astype(int)) > num_of_pixel:
        return 1
    return 0

    '''
    assert(level in [0,1,2,3])
    return level'''

def rateAlertFromDepthCamera(ground_level, baseMat, disparityFrame):
    """
    rightFeatures = []
    leftFeatures = []
    for trackedFeature in trackedFeatures:
        y, x = int(trackedFeature.position.y), int(trackedFeature.position.x)
        if y < 720:
            if x > 640: # right
                rightFeatures.append(trackedFeature)
            else:
                leftFeatures.append(trackedFeature)
    """
    midCol = baseMat.shape[0] // 2
    left_frame = ground_level[:, :midCol]
    right_frame = ground_level[:, midCol:]
    left_baseMat = baseMat[:, :midCol]
    right_baseMat = baseMat[:, midCol:]
    left_level = rateSubframe(left_frame, left_baseMat)
    right_level = rateSubframe(right_frame, right_baseMat)
    return (left_level, right_level)

import timeit
import copy

if __name__ == '__main__':
    '''
    import bluetooth
    target_name = "KeltonAirPods"
    target_addr = None
    devices = bluetooth.discover_devices()
    for bdaddr in nearby_devices:
    if target_name == bluetooth.lookup_name( bdaddr ):
        target_address = bdaddr
        break

    if target_address is not None:
        print "found target bluetooth device with address ", target_address
    else:
        print "could not find target bluetooth device nearby"
    bd_addr =
    port =
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect(bd_addr, port)
    sock.send('\x1A')'''

    james_speak('alex dumb')

    stereoFrame, disparityFrame = getAugmentedFeature()
    timestamp = stereoFrame.getTimestamp()
    stereoFrame = stereoFrame.getFrame()
    # trackedFeatures = trackedFeatures.trackedFeatures
    # calibration
    # 720 x 1280, where valid values are within [350, 5520], ignore 0s
    (rowNum, colNum) = stereoFrame.shape
    D = getDepthThreshold()
    stereoFrame = filterInvalidDepth(stereoFrame, D)
    leftFrame, rightFrame = stereoFrame[:colNum//2], stereoFrame[colNum//2:]
    baseline = np.array(stereoFrame[:, colNum//2], dtype=float)
    x = np.array(range(rowNum))[baseline != 0].reshape((-1, 1))
    baseline = baseline[baseline != 0]
    baseline_reciprocal = 1 / baseline
    reg = Ridge().fit(x, baseline_reciprocal, np.array(range(len(x)))+1)
    pred = 1 / reg.predict(np.array(range(rowNum)).reshape(-1, 1))

    angleStep = (71.9 / 1280) * (np.pi / 180)
    baseMat = np.empty_like(stereoFrame, dtype=float).transpose()
    for i in range(colNum):
        baseMat[i] = getRowBase(pred, i, angleStep)
    baseMat = baseMat.transpose()
    print(f'calibration time : {timestamp}')
    port = glob.glob('/dev/ttyACM*')[0]
    ser = serial.Serial(port, 9600, timeout=1)
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
    ser.write(bytes("modifyVibrator 12 22 32 42 52 62", 'utf-8'))
    # plt.plot(np.flip(baseline))
    # plt.plot(np.flip(pred))
    # plt.xlabel("pixels from near to far")
    # plt.ylabel("depth (millimeter)")

    # baseMat = baseMat.transpose()
    # assert(baseMat.shape==(400, 574))
    #assert(pred.shape==(400,))
    #baselineDict = vecToDict(pred)
    feature_IDs = set()
    print("start main kernel")
    # the same baud rate as the one used on Arduino


    module_indices = [i for i in range(6)]
    historical_readings = [0,0,0,0,0,0]#defaultdict(list)
    sensor_readings =[]

    # distance_threshold = 100
    #for i in range(6):
    #    historical_readings[i] = [0]


    last_intensities = [-1 for i in range(6)]
    cnt = 0
    total = 0
    while True:
    # filming
        start = timeit.default_timer()
        stereoFrame, disparityFrame = getAugmentedFeature()
        stereoFrame = stereoFrame.getFrame()
        disparityFrame = disparityFrame.getFrame()
        #frame = np.zeros(stereoFrame.shape)
        #ground_level_Dict = frameToDict(stereoFrame)
        ground_level = stereoFrame
        left_level, right_level = rateAlertFromDepthCamera(ground_level, baseMat, disparityFrame)
        print(f'left level {left_level}')
        print(f'right level {right_level}')

        while True:
            if ser.in_waiting > 0:
                lines = ser.read(size=1000).decode('utf-8').rstrip()
                print('lines: ' + lines + '\n')
            # data: <reading 0-500> x6
            # msg: <debug msg>
                for line in lines.split('\n'):
                    print("line:", line)
                    if line.startswith('data'):
                        _, *sensor_readings = line.split(' ')
                        assert(len(sensor_readings)==6)
    #                     module_indices.append(module_idx)
                        #historical_readings[module_idx].append(sensor_reading)
                    elif line.startswith('msg'):
                        print(line)
                    elif line.startswith('error'):
                        print(line)
                break

    #     # Rate “threat level” of each identified obstacle based on distance and speed
    #     # Activate vibration system accordingly
    #     # intensities = [0 for i in range(6)]

    #     # modifyVibrator <num 1-6><level 1-3>
    #     # deactivateVibrator <num>
    #     # deactivateSensor
        print(sensor_readings)
        sensor_readings = list(map(int, sensor_readings))
        for module_idx in module_indices:
            if True:#len(historical_readings[module_idx]) >= 1:
                current_reading = sensor_readings[module_idx]
                dx = current_reading - historical_readings[module_idx]
                # 200ms between readings
                # e.g. 1m/s = 200cm/0.2s
                intensity = last_intensities[module_idx]
                if current_reading > 0:
                    if current_reading <= 50:
                        intensity = 2
                    elif current_reading <= 100:
                        intensity = 1
                    elif current_reading <= 200:
                        intensity = 0 if dx <= 70 else 2
                    elif current_reading <= 300:
                        intensity = 0 if dx <= 70 else 1
                    else:
                        intensity = 0
                if module_idx == 2:
                    intensity = max(intensity, left_level)
                elif module_idx == 3:
                    intensity = max(intensity, right_level)

                print(intensity)
                if intensity != last_intensities[module_idx]:
                    last_intensities[module_idx] = intensity
                    ser.write(bytes("modifyVibrator " + str(module_idx) + str(intensity), 'utf-8'))
                    print("modifyVibrator " + str(module_idx) +  str(intensity))
        historical_readings = copy.copy(sensor_readings)
        end = timeit.default_timer()
        cnt += 1
        diff = end - start
        total += diff
        print(total/cnt)

    #     # if input("break ?")=='b':
    #     # if keyboard.read_key():
    #     #     for module_idx in range(1, 7):
    #     #         ser.write(bytes("modifyVibrator " + str(module_idx) +  '0', 'utf-8'))
    #     #         ser.write(bytes("deactivateSensor", 'utf-8'))
    #     #     break