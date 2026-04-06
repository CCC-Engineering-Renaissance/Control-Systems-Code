# Overview

I have made a directory for the camera that includes all the code related to the camera. In this directory there are three files key to using the camera
 and they are "camera.py", "camSend.py", and "configCamera.py". camSend.py is the code that is responsible for how the camera sends the camera footage
 data, configCamera.py is responsible for any configurations you would like to make to the camera, and camera.py is the main that runs the functions from
 camSend.py and uses the configurations set by configCamera.py.

For the camera we chose to use GStreamer as it works well for systems that require multiple sets of cameras such as our ROV which will contain multiple cam>
 cameras.


## camSend.py

### Library/Imports

The library we are using is cv2 and this is mainly used for various camera functions that are key for managing data from the camera. We also are importing
 configCamera.py to allow for ease of customization.

### Class Initialization and Hardware Connection

_init_:a function which sets each camera to a designated port, camera number, and initializes the hardware connection.

Camera Setup: it uses "cv2.VideoCapture" with the "cameraID" and the backend driver(V4L2 for Linux/Raspberry Pi) defined in configCamera.py.
"propertyMap": The class automatically applies settings such as brightness, contrast, and resolution by iterating through the "propertyMap" from
 configCamera.py file.
GStreamer Pipeline: When and if the camera is opened successfully, a Gstreamer string is constructed. The pipeline handles hardware-accelerated H.264
 encoding and packages the video into RTP packets to be sent via UDP to the surface laptop's IP address.
Validation: The "initSuccess" flag is only set to "True" if both the camera hardware and the GStreamer "VideoWriter" are successfully initialized.


### Streaming Logic
The "stream()" function contains the main loop for data transmission:
-> Continuously reads frames from the camera buffer.
-> Each frame is passed to the GStreamer pipeline via "self.out.write(frame)".
-> Resource Management: A "finally" block is included to ensure that even if the program crashes, the camera hardware and video writers are released
 properly to prevent errors on restart.

### "streamConvert" Class
-> "appsrc": Tells GStreamer that our video frames are coming from the python code(OpenCV).
-> "rtph264pay": wraps the encoded video into RTP(Real-time Transport Protocol) packets
-> "udpsink": sends those packets over the network

___________________________________________________________________________________________________________________________________________________________>
## camera.py
This script serves as the entry point for the camera system, utilizing Python's "multiprocessing" library to handle multiple camera feeds simultaneously
 without lag.

### Library/Imports
Threading->used to handle multiple things at once
Time->used to delay program execution
sys->used to handle system level operation

### Multi-Process Architecture
Because processing video is CPU-intensive, we run each camera in its own dedicated process.
-> startCamera(): A wrapper function that instantiates the "cameraROV" class and starts the stream.
-> Main Loop: The script reads the "cameras" dictionary from configCamera.py file and spawns a new process for every entry.
-> Daemon Processes: Processes are set as "daemon = True", meaning they will automatically shut down when the main script exits.

### Shutdown Procedure
To prevent orphaned processes, the script catches a "KeyboardInterrupt" (Ctrl+C). When triggered, it explicitly calls ".terminate()" and ".join()" on
 every active camera process to ensure a clean exit.

___________________________________________________________________________________________________________________________________________________________>

## configCamera.py

This file is the central hub for all hardware and network adjustments. It is designed so that the user does not need to touch the core logic in the other
 files to make common changes.

### Configuration Categories
-> Video settings: controls the resolution and fluidity of the stream
-> Image Quality: Tunes the visual output
-> Encoding: allows us to switch between software encoding
-> Network: Defines where the data is sent across the tether
-> Camera Map: a dictionary mapping cameras to their physical USB IDs and network ports

## Troubleshooting
When running multiple USB cameras over a network using GStreamer can occasionally run into issues. The most common issues and how to resolve them
 using the configCamera.py file:

Device Busy Error -> Run "fuser -k /dev/video*" in the terminal or restart the Pi to release the hardware lock.
High Latency(Lag) -> Lower the "bitrate" in the "encoder" string or reduce "framesPerSecond" to 15 or 20.
Grey/Broken Frames -> Ensure "laptopIPAddress" is correct and check if "h264_i_frame_period" is set to a low value (1-2) for faster recovery.
Camera Not Found -> Update the "id" in the "cameras" dictionary within configCamera.py

___________________________________________________________________________________________________________________________________________________________>

## Network Setup Requirements
For the ROV and the Laptop to communicate, they must be on the same subnet.
        1. Static IP: It is recommended to set a static IP for the Raspberry Pi.
        2. Subnet Mask: Ensure both devices use the same mask.
        3. Firewall: On the Laptop, ensure that the ports(5000-5004) are open in the Windows/Linux firewall to allow incoming UDP signals.

## How to Run
        1. Ensure V4L2 is supported
        2. Install GStreamer and Python Bindings
                OpenCV needs to be linked with GStreamer
        3. Connect all USB cameras to the Raspberry Pi
        4. Edit configCamera.py to verify the ROV's IP Address, Camera IDs, and Encoder
        5. Open a terminal in the project directory
        6. In Terminal Run:
		cd Control-Systems-Code
		To launch: make -C camera/ run
		To stop: make -C camera/ stop



