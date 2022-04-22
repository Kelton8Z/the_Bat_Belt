#!/usr/bin/env python3
# import serial
import cv2
import numpy as np
from oak_d_test import getAugmentedFeature
# import oak_d_test
from collections import defaultdict
from sklearn.linear_model import Ridge

import matplotlib.pyplot as plt

import numpy as np

def getDepthThreshold():
    BL = 75 #mm
    HFOV = 72 #deg
    Dv = (BL / 2) * np.tan((90 - HFOV / 2) * np.pi / 180)
    W = 1280 #720p
    D = np.empty(W)
    numerator = W * Dv
    for B in range(W // 2):
        D[B] = numerator / B
        D[-(B + 1)] = D[B]
    return D
    
def filterInvalidDepth(stereoFrame, D):
    row, col = stereoFrame.shape
    for i in range(row):
        for j in range(col):
            if stereoFrame[i][j] < D[j]:
                stereoFrame[i][j] = 0 
    return stereoFrame

def getRowBase(midBase, col, angleStep, midCol = 287, HFOV = 71.9):
    colDiff = np.abs(col - midCol)
    return midBase / np.cos(colDiff * angleStep)

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


def rateSubframe(ground_level, baseMat, features):
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
        top_score = normalize(top_diffs)
    if mid_diffs:
        mid_score = normalize(mid_diffs) 
    if bottom_diffs:
        bottom_score = normalize(bottom_diffs)

    level = (top_score + 2*mid_score + 3*bottom_score)//6
    print(top_diff)
    print(mid_diff)
    print(bottom_diff)
    #print(level)
    assert(level in [0,1,2,3])
    return level

def rateAlertFromDepthCamera(ground_level, baseMat, trackedFeatures):
    rightFeatures = []
    leftFeatures = []
    for trackedFeature in trackedFeatures:
        y, x = int(trackedFeature.position.y), int(trackedFeature.position.x) 
        if x < 720:
            if y > 640: # right
                rightFeatures.append(trackedFeature)
            else:
                leftFeatures.append(trackedFeature)

    right_level = rateSubframe(ground_level, baseMat, leftFeatures)
    left_level = rateSubframe(ground_level, baseMat, rightFeatures)
    return (left_level, right_level)

if __name__ == '__main__':         
    rgbFrame, stereoFrame, trackedFeatures = getAugmentedFeature()
    timestamp = stereoFrame.getTimestamp()
    rgbFrame, stereoFrame = rgbFrame.getFrame()[:720], stereoFrame.getFrame()
    # trackedFeatures = trackedFeatures.trackedFeatures
    # calibration
    print(f'calibration time : {timestamp}')
    # 720 x 1280, where valid values are within [350, 5520], ignore 0s
    (rowNum, colNum) = stereoFrame.shape
    D = getDepthThreshold()
    stereoFrame = filterInvalidDepth(stereoFrame, D)
    leftFrame, rightFrame = stereoFrame[:colNum//2], stereoFrame[colNum//2:]
    baseline = np.array(stereoFrame[:, colNum//2], dtype=float)
    x = np.array(range(rowNum))[baseline != 0].reshape((-1, 1))
    #baseline = baseline[baseline != 0]
    baseline_reciprocal = 1 / baseline
    #reg = Ridge().fit(x, baseline_reciprocal, np.array(range(len(x)))+1)
    #pred = 1 / reg.predict(np.array(range(rowNum)).reshape(-1, 1))

    angleStep = (71.9 / 640) * (np.pi / 180)
    baseMat = np.empty_like(stereoFrame, dtype=float).transpose()
    for i in range(colNum):
        baseMat[i] = baseline # getRowBase(pred, i, angleStep)

    # plt.plot(np.flip(baseline))
    # plt.plot(np.flip(pred))
    # plt.xlabel("pixels from near to far")
    # plt.ylabel("depth (millimeter)")

    # baseMat = baseMat.transpose()
    # assert(baseMat.shape==(400, 574))
    #assert(pred.shape==(400,))
    #baselineDict = vecToDict(pred)
    feature_IDs = set()
    while True:
    # filming
        rgbFrame, stereoFrame, trackedFeatures = getAugmentedFeature()
        rgbFrame, stereoFrame = rgbFrame.getFrame()[:720], stereoFrame.getFrame()
        trackedFeatures = trackedFeatures.trackedFeatures
        errors = []
        #frame = np.zeros(stereoFrame.shape)
        #ground_level_Dict = frameToDict(stereoFrame)
        ground_level = stereoFrame
        left_level, right_level = rateAlertFromDepthCamera(ground_level, baseMat, trackedFeatures)
        print(f'left level {left_level}')
        print(f'right level {right_level}')

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

    ser.write(b"activateSensor")
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            # data: <reading 0-500> x6
            # msg: <debug msg>
            print(line)
            if line.startswith('data'):
                _, sensor_readings = line.split(' ')
                for module_idx, sensor_reading in enumerate(sensor_readings):
                    sensor_reading = int(sensor_reading)
#                     module_indices.append(module_idx)
                    historical_readings[module_idx].append(sensor_reading)
            elif line.startswith('msg'):
                pass
            elif line.startswith('error'):
                print(line)


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
