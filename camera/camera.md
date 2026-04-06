# Overview

I have made a directory for the camera that includes all the code related to the camera. In this directory there are three files key to using the camera and they are "camera.py", "camSend.py", and "configCamera.py". camSend.py is the code that is responsible for how the camera sends the camera footage data, configCamera.py is responsible for any configurations you would like to make to the camera, and camera.py is the main that runs the functions from camSend.py and uses the configurations set by configCamera.py.

For the camera we chose to use GStreamer as it works well for systems that require multiple sets of cameras such as our ROV which will contain multiple cameras. Since the camera we are working with is connected through USB, we had to implement appropriate protocols that were compatible with such cameras.

## camSend.py 


### Library/Imports

The library we are using is cv2 and this is mainly used for various camera functions that are key for managing data from the camera. We also are importing configCamera.py to allow for ease of customization.

### Class Initialization and Hardware Connection

We created a function __init__ which sets each camera to a designated port, camera number, and  





