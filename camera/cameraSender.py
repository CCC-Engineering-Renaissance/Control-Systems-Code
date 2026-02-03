import cv2
import configCamera as config 
import threading

class cameraROV:
    def __init__(self, cameraID, port):
        self.cameraID = cameraID #tells you the camera number
        self.port = port # what port the respective camera gets sent to 
        self.cap = cv2.VideoCapture(self.cameraID)

        if not self.cap.isOpened():
            print("Error")
        
        
        #this is the gstreamer pipeline string that converts the frames from the if statement into a stream in the network
        #if video is blurry/pixelated increase bitrate (you can do this in configCamera.py)
        #if video is laggy/delayed lower bitrate
        #if pi is getting hot ensure ultrafast is on and maybe lower resolution
        streamConvert = (
            f"appsrc ! videoconvert ! x264enc tune={config.tunePreset} speed-preset={config.speedPreset} bitrate={config.bitRate} ! "
            f"rtph264pay config-interval=1 pt=96 ! "
            F"queue ! "
            f"udpsink host={config.laptopIPAddress} port={self.port}"
        )
        
        #tells the computer how to prepare for the data sent from streamConvert
        self.out = cv2.VideoWriter(streamConvert, cv2.CAP_GSTREAMER, 0, 30, (config.frameWidth, config.frameHeight))
        
        #should print when completed the function per each camera
        print(f"Setup complete for {self.cameraID}")
    
        #basically acts like a stop motion film but that is how we are able to send the camera data to the computer 
    def stream(self):
        while True: 
            ret, frame = self.cap.read() #takes the photo

            if ret:
                self.out.write(frame) #sends photo to laptop
            else:
                print(f"Error: Camera {self.cameraID} disconnected.")
                break

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        #only happens when loop breaks   
        self.cap.release()
        self.out.release()

