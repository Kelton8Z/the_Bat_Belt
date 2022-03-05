# first, import all necessary modules
from pathlib import Path

import blobconverter
import cv2
import depthai as dai
import numpy as np

# Closer-in minimum depth, disparity range is doubled (from 95 to 190):
extended_disparity = False
# Better accuracy for longer distance, fractional disparity 32-levels:
subpixel = True
# Better handling for occlusions:
lr_check = False

# Pipeline tells DepthAI what operations to perform when running - you define all of the resources used and flows here
pipeline = dai.Pipeline()

# First, we want the Color camera as the output
# cam_rgb = pipeline.createColorCamera()
# cam_rgb.setPreviewSize(300, 300)  # 300x300 will be the preview frame size, available as 'preview' output of the node
# cam_rgb.setInterleaved(False)

# # Next, we want a neural network that will produce the detections
# detection_nn = pipeline.createMobileNetDetectionNetwork()
# # Blob is the Neural Network file, compiled for MyriadX. It contains both the definition and weights of the model
# # We're using a blobconverter tool to retreive the MobileNetSSD blob automatically from OpenVINO Model Zoo
# detection_nn.setBlobPath(blobconverter.from_zoo(name='mobilenet-ssd', shaves=6))
# # Next, we filter out the detections that are below a confidence threshold. Confidence can be anywhere between <0..1>
# detection_nn.setConfidenceThreshold(0.5)
# # Next, we link the camera 'preview' output to the neural network detection input, so that it can produce detections
# cam_rgb.preview.link(detection_nn.input)

# XLinkOut is a "way out" from the device. Any data you want to transfer to host need to be send via XLink
# xout_rgb = pipeline.createXLinkOut()
# # For the rgb camera output, we want the XLink stream to be named "rgb"
# xout_rgb.setStreamName("rgb")
# Linking camera preview to XLink input, so that the frames will be sent to host
# cam_rgb.preview.link(xout_rgb.input)

# The same XLinkOut mechanism will be used to receive nn results
# xout_nn = pipeline.createXLinkOut()
# xout_nn.setStreamName("nn")
# detection_nn.out.link(xout_nn.input)

# Pipeline is now finished, and we need to find an available device to run our pipeline
# we are using context manager here that will dispose the device after we stop using it
# From this point, the Device will be in "running" mode and will start sending data via XLink

# To consume the device results, we get two output queues from the device, with stream names we assigned earlier
# q_rgb = device.getOutputQueue("rgb")
# q_nn = device.getOutputQueue("nn")


# Here, some of the default values are defined. Frame will be an image from "rgb" stream, detections will contain nn results
frame = None
detections = []

camRgb = pipeline.create(dai.node.ColorCamera)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)

edgeDetectorLeft = pipeline.create(dai.node.EdgeDetector)
edgeDetectorRight = pipeline.create(dai.node.EdgeDetector)
edgeDetectorRgb = pipeline.create(dai.node.EdgeDetector)

xoutEdgeLeft = pipeline.create(dai.node.XLinkOut)
xoutEdgeRight = pipeline.create(dai.node.XLinkOut)
xoutEdgeRgb = pipeline.create(dai.node.XLinkOut)
xinEdgeCfg = pipeline.create(dai.node.XLinkIn)

edgeLeftStr = "edge left"
edgeRightStr = "edge right"
edgeRgbStr = "edge rgb"
edgeCfgStr = "edge cfg"

xoutEdgeLeft.setStreamName(edgeLeftStr)
xoutEdgeRight.setStreamName(edgeRightStr)
xoutEdgeRgb.setStreamName(edgeRgbStr)
xinEdgeCfg.setStreamName(edgeCfgStr)

# Properties
camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

edgeDetectorRgb.setMaxOutputFrameSize(camRgb.getVideoWidth() * camRgb.getVideoHeight())

# Linking
monoLeft.out.link(edgeDetectorLeft.inputImage)
monoRight.out.link(edgeDetectorRight.inputImage)
camRgb.video.link(edgeDetectorRgb.inputImage)

edgeDetectorLeft.outputImage.link(xoutEdgeLeft.input)
edgeDetectorRight.outputImage.link(xoutEdgeRight.input)
edgeDetectorRgb.outputImage.link(xoutEdgeRgb.input)

xinEdgeCfg.out.link(edgeDetectorLeft.inputConfig)
xinEdgeCfg.out.link(edgeDetectorRight.inputConfig)
xinEdgeCfg.out.link(edgeDetectorRgb.inputConfig)

depth = pipeline.create(dai.node.StereoDepth)
xout = pipeline.create(dai.node.XLinkOut)

xout.setStreamName("disparity")



# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth.setLeftRightCheck(lr_check)
depth.setExtendedDisparity(extended_disparity)
depth.setSubpixel(subpixel)

# Linking
monoLeft.out.link(depth.left)
monoRight.out.link(depth.right)
depth.disparity.link(xout.input)

device = dai.Device(pipeline)
# Output/input queues
edgeLeftQueue = device.getOutputQueue(edgeLeftStr, 8, False)
edgeRightQueue = device.getOutputQueue(edgeRightStr, 8, False)
edgeRgbQueue = device.getOutputQueue(edgeRgbStr, 8, False)
edgeCfgQueue = device.getInputQueue(edgeCfgStr)

q = device.getOutputQueue(name="disparity", maxSize=4, blocking=False)

# Since the detections returned by nn have values from <0..1> range, they need to be multiplied by frame width/height to
# receive the actual position of the bounding box on the image
def frameNorm(frame, bbox):
    normVals = np.full(len(bbox), frame.shape[0])
    normVals[::2] = frame.shape[1]
    return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

def getDisparityFrame():
    inDisparity = q.get()  # blocking call, will wait until a new data has arrived
    frame = inDisparity.getFrame()
    # Normalization for better visualization
    frame = (frame * (255 / depth.initialConfig.getMaxDisparity())).astype(np.uint8)
    return frame

def getEdgeFrames():
    edgeLeft = edgeLeftQueue.get()
    edgeRight = edgeRightQueue.get()
    edgeRgb = edgeRgbQueue.get()

    edgeLeftFrame = edgeLeft.getFrame()
    edgeRightFrame = edgeRight.getFrame()
    edgeRgbFrame = edgeRgb.getFrame()
    edgeRgbFrame = (edgeRgbFrame * (255 / depth.initialConfig.getMaxDisparity())).astype(np.uint8)
    return edgeLeftFrame, edgeRightFrame, edgeRgbFrame

