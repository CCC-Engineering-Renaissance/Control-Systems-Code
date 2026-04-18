import cv2
import platform

#for video stream
framesPerSecond = 30
frameWidth = 640
frameHeight = 480
videoFormat = cv2.VideoWriter_fourcc(*'MJPG')
bufferSize = 1

#for video view settings
brightness = 50 
contrast = 50
saturation = 60

# use encoder below for pi
encoder = "v4l2h264enc extra-controls=\"controls,video_bitrate=1500000,h264_i_frame_period=1\""  #for pi
#use encoder below for laptop testing
#encoder = "x264enc bitrate=2000 tune=zerolatency speed-preset=ultrafast" #for laptop

# CAP_DSHOW = Windows, CAP_V4L2 = Linux/Pi
backend = cv2.CAP_DSHOW  # use this for testing on Windows
#backend = cv2.CAP_V4L2  # use this for Pi/Linux on the ROV

guid = None
bitRate = 2000000
timeOutInterval = 5000
frameBytes = 0
quality = 95

#the network
cameras = {
    "camera0": {"id": 0, "port": 5000},
    "camera1": {"id": 1, "port": 5001},
    "camera2": {"id": 2, "port": 5002},
    "camera3": {"id": 3, "port": 5003},
    "camera4": {"id": 4, "port": 5004},
}

laptopIPAddress = "172.30.64.1"
chunkSize = 1024

#property mapping
propertyMap = {
    cv2.CAP_PROP_BRIGHTNESS: brightness,
    cv2.CAP_PROP_CONTRAST: contrast,
    cv2.CAP_PROP_SATURATION: saturation,
    cv2.CAP_PROP_FRAME_WIDTH: frameWidth,
    cv2.CAP_PROP_FRAME_HEIGHT: frameHeight,
    cv2.CAP_PROP_FPS: framesPerSecond,
    cv2.CAP_PROP_BUFFERSIZE: bufferSize,
    cv2.CAP_PROP_FOURCC: videoFormat
}
