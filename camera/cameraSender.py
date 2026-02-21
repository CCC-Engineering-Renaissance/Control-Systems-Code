import cv2
import configCamera as config 
import threading



#script requires gstreamer and its plugins to be installed on pi DONT FORGET

class cameraROV:
    def __init__(self, cameraID, port):

        #making flag to make sure code only runs for a respective camera if it detects it exists
        self.initSuccess = False
        self.cameraID = cameraID #tells you the camera number
        self.port = port # what port the respective camera gets sent to 
<<<<<<< HEAD
        self.cap = cv2.VideoCapture(self.cameraID, config.backend)
=======
        self.cap = cv2.VideoCapture(self.cameraID) #connects to the physical USB camera
>>>>>>> Safe_Space
        
        #sets all properties in configCamera.py prior to capturing anything!
        for prop, value in config.propertyMap.items():
                self.cap.set(prop, value)


        ret, frame = self.cap.read()
<<<<<<< HEAD

        if self.cap.isOpened() and ret:
            print(f"Camera {self.cameraID} is open and sending data")

        #this is the gstreamer pipeline string that converts the frames from the if statement into a stream in the network
        #if video is blurry/pixelated increase bitrate (you can do this in configCamera.py)
        #if video is laggy/delayed lower bitrate
        #if pi is getting hot ensure ultrafast is on and maybe lower resolution
        #streamCovert here is what we should actually be using but for testing im making another version of this to work with the testing encoder 

        streamConvert = (
             f"appsrc ! videoconvert ! "
             f"{config.encoder} ! "
             f"rtph264pay config-interval=1 pt=96 ! "
             f"queue ! "
             f"udpsink host={config.laptopIPAddress} port={self.port}"
         )

        #tells the computer how to prepare for the data sent from streamConvert
        self.out = cv2.VideoWriter(streamConvert, cv2.CAP_GSTREAMER, 0, 30, (config.frameWidth, config.frameHeight))

        #when the code prior to this succeeds it will signal a true flag instead of false
        #this way the code wont just send data to a camera that isnt even being detected

        self.initSuccess = True 
        #should print when completed the function per each camera
        print(f"Setup complete for {self.cameraID}")
=======
        if self.cap.isOpened() and ret: #checks if camera is connected/sending data
            print(f"Camera {self.cameraID} is open and sending data")
            for prop, value in config.propertyMap.items(): #loops through resolution, etc.
                self.cap.set(prop, value) #applies setting to hardware
           
            #this is the gstreamer pipeline string that converts the frames from the if statement into a stream in the network
            #if video is blurry/pixelated increase bitrate (you can do this in configCamera.py)
            #if video is laggy/delayed lower bitrate
            #if pi is getting hot ensure ultrafast is on and maybe lower resolution
            streamConvert = (
                f"appsrc ! videoconvert ! x264enc tune={config.tunePreset} speed-preset={config.speedPreset} bitrate={config.bitRate} ! "
                f"rtph264pay config-interval=1 pt=96 ! "
                f"queue ! "
                f"udpsink host={config.laptopIPAddress} port={self.port}"
            ) #'x264enc' encoding may need to be changed
        
            #tells the computer how to prepare for the data sent from streamConvert
            self.out = cv2.VideoWriter(streamConvert, cv2.CAP_GSTREAMER, 0, 30, (config.frameWidth, config.frameHeight))
        
            #when the code prior to this succeeds it will signal a true flag instead of false
            #this way the code wont just send data to a camera that isnt even being detected
        
            self.initSuccess = True 
            #should print when completed the function per each camera
            print(f"Setup complete for {self.cameraID}")
>>>>>>> Safe_Space


        else:
            print(f"Error: initial setup for camera {self.cameraID} failed")
        

    
        #basically acts like a stop motion film but that is how we are able to send the camera data to the computer 
    def stream(self):
        try: 
            if not self.initSuccess: #if setup failed, don't try to stream
                raise Exception("Initialization was False")

            while True: 
                ret, frame = self.cap.read() #takes the photo

                if ret:
<<<<<<< HEAD
                    #the code directly under this comment was put to help for testing the camera on my laptop!
                    cv2.imshow(f"ROV Camera {self.cameraID} Test", frame)
                    self.out.write(frame)
=======
                    self.out.write(frame) #sends the frame through GStreamer to laptop
>>>>>>> Safe_Space
                else:
                    print(f"Error: Camera {self.cameraID} disconnected.")
                    break
                #press q to quit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break


        except Exception as e:
            print(f"Stream blocked for Camera {self.cameraID}: {e}")


        #only happens when loop breaks

        #grabs image data
        self.cap.release()
        if self.initSuccess:
            self.out.release() #converts image data
            
