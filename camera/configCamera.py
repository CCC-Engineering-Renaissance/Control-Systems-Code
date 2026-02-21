import cv2



#for video stream
framesPerSecond = 30 #how much frames we want displayed to surface side, may need to be lowered
frameWidth = 640
frameHeight = 480
videoFormat = cv2.VideoWriter_fourcc(*'MJPG') #the four characters that define what file type the camera uses for formatting
#we use MJPG do we can  get jpeg as our video format- again this can be changed to your liking
bufferSize = 1 #number of frames held in the internal buffer- maybe the bucket??

#for video view settings whatever surface side wants to add or get rid of you can do. i just made this for it to be easier to change for when displaying
brightness = 50 
contrast = 50
saturation = 60




encoder = "v4l2h264enc extra-controls=\"controls,video_bitrate=2000000,h264_i_frame_period=1\"" 
#for testing on laptop we may need to use msdkh264enc
# for the pi on the rov we will use v4l2h264enc so change to x264enc for the actual rov




# Change from cv2.CAP_V4L2 to cv2.CAP_DSHOW for Windows testing
#backend = cv2.CAP_DSHOW use this for testing on windows

<<<<<<< HEAD
backend = cv2.CAP_V4L2 #tells you which driver is being used?


=======
backend = cv2.CAP_V4L2 #uses standard linux driver for video
>>>>>>> Safe_Space
guid = None #unique identifier for camera hardware
bitRate = 2000000 #data rate of stream, this is about 2Mbps
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

laptopIPAddress = "127.0.0.1" #will use this ip address on laptop to see footage
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



