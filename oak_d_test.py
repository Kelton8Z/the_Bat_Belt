# first, import all necessary modules
from pathlib import Path

import blobconverter
import cv2
import depthai as dai
import numpy as np

# Closer-in minimum depth, disparity range is doubled (from 95 to 190):
extended_disparity = True
# Better accuracy for longer distance, fractional disparity 32-levels:
subpixel = False
# Better handling for occlusions:
lr_check = True

frame = None
detections = []

pipeline = dai.Pipeline()

monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
edgeDetector = pipeline.create(dai.node.EdgeDetector)
edgeOut = pipeline.create(dai.node.XLinkOut)
depthOut = pipeline.create(dai.node.XLinkOut)

# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
stereo.setLeftRightCheck(lr_check)
stereo.setExtendedDisparity(extended_disparity)
stereo.setSubpixel(subpixel)

cfg = stereo.initialConfig.get()
# cfg.postProcessing.speckleFilter.enable = True
# cfg.postProcessing.spatialFilter.enable = True
cfg.postProcessing.thresholdFilter.minRange = 349
# cfg.postProcessing.thresholdFilter.maxRange = 5758
stereo.initialConfig.set(cfg)

sobelHorizontalKernel = [[1, 0, -1], [2, 0, -2], [1, 0, -1]]
sobelVerticalKernel = [[1, 2, 1], [0, 0, 0], [-1, -2, -1]]
edgeDetector.initialConfig.setSobelFilterKernels(sobelHorizontalKernel, sobelVerticalKernel)

edgeOut.setStreamName("edge")
depthOut.setStreamName("depth")

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)
stereo.disparity.link(edgeDetector.inputImage)
edgeDetector.outputImage.link(edgeOut.input)
stereo.depth.link(depthOut.input)

device = dai.Device(pipeline)
# Output/input queues
edgeQueue = device.getOutputQueue(name="edge", maxSize=4, blocking=False)
depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

def getDepthFrame():
    return depthQueue.get()

def getEdgeFrame():
    return edgeQueue.get().getFrame()

