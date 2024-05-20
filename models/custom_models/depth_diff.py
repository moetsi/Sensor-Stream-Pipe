import numpy as np
import cv2
import depthai as dai


resolution = (1632,960) # 24 FPS (without visualization)
lrcheck = True  # Better handling for occlusions
extended = False  # Closer-in minimum depth, disparity range is doubled
subpixel = True  # True  # Better accuracy for longer distance, fractional disparity 32-levels

p = dai.Pipeline()

# Configure Mono Camera Properties
left = p.createMonoCamera()
left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
left.setBoardSocket(dai.CameraBoardSocket.LEFT)

right = p.createMonoCamera()
right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
right.setBoardSocket(dai.CameraBoardSocket.RIGHT)



stereo = p.createStereoDepth()
left.out.link(stereo.left)
right.out.link(stereo.right)

camRgb = p.createColorCamera()
camRgb.setPreviewSize(1632, 960)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

# now we link out both the left and right
# leftout = p.createXLinkOut()
# leftout.setStreamName("left")
# rightout = p.createXLinkOut()
# rightout.setStreamName("right")
# left.out.link(leftout.input)
# right.out.link(rightout.input)
# Set stereo depth options
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
config = stereo.initialConfig.get()
config.postProcessing.speckleFilter.enable = True
config.postProcessing.speckleFilter.speckleRange = 60
config.postProcessing.temporalFilter.enable = True

config.postProcessing.spatialFilter.holeFillingRadius = 2
config.postProcessing.spatialFilter.numIterations = 1
config.postProcessing.thresholdFilter.minRange = 700  # mm
config.postProcessing.thresholdFilter.maxRange = 7000  # mm
config.censusTransform.enableMeanMode = True
config.costMatching.linearEquationParameters.alpha = 0
config.costMatching.linearEquationParameters.beta = 2
stereo.initialConfig.set(config)
stereo.setLeftRightCheck(lrcheck)
stereo.setExtendedDisparity(extended)
stereo.setSubpixel(subpixel)
stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
stereo.setRectifyEdgeFillColor(0)  # Black, to better see the cutout

# Now we create a script node that takes in the depth frames, waits 5 seconds, and sends it to the nn
# The nn will then output the depth diff
# Depth -> Script -> NN -> XLinkOut
pause_and_print_script = p.create(dai.node.Script)
stereo.depth.link(pause_and_print_script.inputs["depth_in"])
pause_and_print_script.setScript("""
import time

timeStart = time.time()
sentOnce = False

while True:
    time.sleep(.001) # avoid lazy loading
    depth = node.io['depth_in'].get()
    # Now we wait 5 seconds and send a single frame once
    if time.time() - timeStart > 1:
        sentOnce = True
        node.io['depth_out'].send(depth)
""")

# Depth -> Depth Diff
nn = p.createNeuralNetwork()
nn.setBlobPath("diff_images_simplified_openvino_2021.4_4shave.blob")
# stereo.depth.link(nn.inputs["input1"])
pause_and_print_script.outputs["depth_out"].link(nn.inputs["input1"])

# Now we create another Script node that takes in the output of the nn and sends it to the host
# NN -> Script -> XLinkOut
depthDiffScript = p.create(dai.node.Script)
nn.out.link(depthDiffScript.inputs["depth_diff_in"])
depthDiffScript.setScript("""
import time
while True:
    time.sleep(.001) # avoid lazy loading
    depthDiff = node.io['depth_diff_in'].get()
    node.io['depth_diff_out'].send(depthDiff)
""")

depthDiffOut = p.createXLinkOut()
depthDiffOut.setStreamName("depth_diff")
# nn.out.link(depthDiffOut.input)
depthDiffScript.outputs["depth_diff_out"].link(depthDiffOut.input)

# We also output the stereo directly to the host by creating another xlink out
depthOut = p.createXLinkOut()
depthOut.setStreamName("depth")
pause_and_print_script.outputs["depth_out"].link(depthOut.input)


device_info = dai.DeviceInfo('18443010C1FC130900')

with dai.Device(p, device_info) as device:
    qDepthDiff = device.getOutputQueue(name="depth_diff", maxSize=4, blocking=True)
    qDepth = device.getOutputQueue(name="depth", maxSize=4, blocking=True)
    # leftQueue = device.getOutputQueue(name="left", maxSize=4, blocking=False)
    # rightQueue = device.getOutputQueue(name="right", maxSize=4, blocking=False)
    while True:

        if qDepthDiff.has() and qDepth.has():
            depthDiff = qDepthDiff.get()
            print("depthDiff.getSequenceNum(): ", depthDiff.getSequenceNum())

            # Shape it here
            floatVector = depthDiff.getFirstLayerFp16()
            diff = np.array(floatVector)

            # Now we print the first 1632 values of the first row of the depth frame
            # print("diff values")
            # print(diff[:1632])
            # Now we print the NEXT 1632 values after the last we printed with no overlap
            # print(diff[1632:1632*2])
            diff = diff.astype(np.uint16)
            diff = diff.reshape(1080, 1920)
            
            colorize = cv2.normalize(diff, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
            cv2.applyColorMap(colorize, cv2.COLORMAP_JET)
            cv2.imshow("Diff", colorize)

            # Now we get the and display the depth
            depth = qDepth.get()
            depthFrame = depth.getCvFrame()

            # Now we print the first 1632 values of the first row of the depth frame
            # print("depth values")
            # print(depthFrame[0][:1632])
            # Now we print the NEXT 1632 values after the last we printed with no overlap
            # print(depthFrame[959][:1632])
            # print(len(depthFrame[0]))
            # print(len(depthFrame))

            depthFrameCv = depth.getCvFrame()
            depthFrameCv = depthFrameCv.astype(np.uint16)
            depthFrame = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
            cv2.imshow("Depth", depthFrame)

            if cv2.waitKey(1) == ord('q'):
                        break

# diff = (diff * (255 / stereo.initialConfig.getMaxDisparity())).astype(np.uint8)

# Create output
# xout = p.createXLinkOut()
# xout.setStreamName("disparity")
# stereo.stereo.link(xout.input)

# # Connect to device and start pipeline
# with dai.Device(p) as device:

#     # Output queue will be used to get the disparity frames from the outputs defined above
#     q = device.getOutputQueue(name="disparity", maxSize=4, blocking=False)

#     while True:
#         inDisparity = q.get()  # blocking call, will wait until a new data has arrived
#         frame = inDisparity.getFrame()
#         # Normalization for better visualization
#         frame = (frame * (255 / stereo.initialConfig.getMaxDisparity())).astype(np.uint8)

#         cv2.imshow("disparity", frame)

#         # Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
#         frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
#         cv2.imshow("disparity_color", frame)

#         if cv2.waitKey(1) == ord('q'):
#             break