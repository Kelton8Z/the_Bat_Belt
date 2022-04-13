import cv2
import depthai as dai

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
featureTracker = pipeline.create(dai.node.FeatureTracker)

xoutRgbFrame = pipeline.create(dai.node.XLinkOut)
xoutStereoDepth = pipeline.create(dai.node.XLinkOut)
xoutTrackedFeatures = pipeline.create(dai.node.XLinkOut)

xoutRgbFrame.setStreamName("rgbFrame")
xoutStereoDepth.setStreamName("stereoDepth")
xoutTrackedFeatures.setStreamName("trackedFeatures")

# Properties
camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setIspScale(2, 3) # downscale to 720P
camRgb.setFps(30)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoLeft.setFps(30)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
monoRight.setFps(30)

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
stereo.setLeftRightCheck(True)
stereo.setExtendedDisparity(True)
stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
cfg = stereo.initialConfig.get()
cfg.postProcessing.thresholdFilter.minRange = 349
stereo.initialConfig.set(cfg)

numShaves = 2
numMemorySlices = 2
featureTracker.setHardwareResources(numShaves, numMemorySlices)

# Linking
camRgb.isp.link(featureTracker.inputImage)
camRgb.isp.link(xoutRgbFrame.input)
featureTracker.passthroughInputImage.link(xoutRgbFrame.input)
featureTracker.outputFeatures.link(xoutTrackedFeatures.input)

monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)
stereo.depth.link(xoutStereoDepth.input)

# Connect to device and start pipeline
device = dai.Device(pipeline)

rgbFrameQueue = device.getOutputQueue("rgbFrame", 8, False)
stereoDepthQueue = device.getOutputQueue("stereoDepth", 8, False)
trackedFeaturesQueue = device.getOutputQueue("trackedFeatures", 8, False)

def getAugmentedFeature():
    print("start getting features") 
    rgbFrame = rgbFrameQueue.get() # 1080 x 1280, use the first 720 rows 
    stereoFrame = stereoDepthQueue.get() # 720 x 1280
    trackedFeatures = trackedFeaturesQueue.get() # trackedFeatures.trackedFeatures gives a list of TrackedFeature objects with feature id, and position
    print(rgbFrame, stereoFrame, trackedFeatures)
    return (rgbFrame, stereoFrame, trackedFeatures)
    # latestPacket = {}
    # queueEvents = device.getQueueEvents(("rgbFrame", "trackedFeatures"))
    # for queueName in queueEvents:
    #     print(queueName)
    #     packets = device.getOutputQueue(queueName).tryGetAll()
    #     if len(packets) > 0:
    #         latestPacket[queueName] = packets[-1]
    # return latestPacket

