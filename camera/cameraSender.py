import cv2
import configCamera as config 
import threading

class cameraROV:
    def __init__(self, cameraID, port):
        self.cameraID = cameraID #tells you the camera number
        self.port = port # what port the respective camera gets sent to 
        self.cap = cv2.VideoCapture(self.cameraID)
        
        #setting resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.frameWidth)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.frameHeight)
        
        #should print when completed the function per each camera
        print(f"Setup complete for {self.cameraID}")
    
    #i will get back to this marking this as where i left off i need to add the while loop and more stuff...
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
