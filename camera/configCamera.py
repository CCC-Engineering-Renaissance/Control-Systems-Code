import cv2

#for video stream
framesPerSecond = 30 #how much frames we want displayed to surface side
frameWidth = 640
frameHeight = 480
videoFormat =cv2.VideoWriter_fourcc(*'MJPG') #the four characters that define what file type the camera uses for formatting
#we use MJPG do we can  get jpeg as our video format- again this can be changed to your liking
bufferSize = 1 #number of frames held in the internal buffer- maybe the bucket??

#for video view settings
brightness = 50
contrast = 50
saturation = 60


backend = cv2.CAP_V4L2 #tells you which driver is being used?
guid = None #unique identifier for camera hardware
bitRate = 1000000 #data rate of stream
timeOutInterval = 5000 #in milliseconds

#for when saving video files
frameBytes = 0 #size of last encoded frame in bytes- we are initializing to zero
#we initilaize to 0 as it is a value that openCV calculates after a frame is processed
quality = 95 #1-100 scale with 100 being the best quality

#the network
cameras = {
    #rename these later once you find the camera placements so these are just temporary names
    #i plan to make the names describe the camera's placement
    "camera0": {"id": 0, "port": 5000},
    "camera1": {"id": 1, "port": 5001},
    "camera2": {"id": 2, "port": 5002},
    "camera3": {"id": 3, "port": 5003},
    "camera4": {"id": 4, "port": 5004},
}

laptopIPAddress = "192.168.1.100" #will use this ip address on laptop to see footage
# also make sure laptop and raspberry pi have an ip address with the first 6 digits matching the laptop's 

chunkSize = 1024 #how much bites of data we group together per one "packet"

#property mapping- since it will make adjusting these features much easier

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



