import cv2
import configCamera as config

class cameraROV:
    def __init__(self, cameraID, port): 
        self.initSuccess = False
        self.cameraID = cameraID
        self.port = port
        self.cap = None
        self.out = None 

        self.cap = cv2.VideoCapture(self.cameraID, config.backend) 
        
        if self.cap.isOpened():
            print(f"Cam {self.cameraID} successfully opened")

            for prop, value in config.propertyMap.items():
                self.cap.set(prop, value)

            streamConvert = (
                f"appsrc ! videoconvert ! {config.encoder} ! "
                f"rtph264pay config-interval=1 pt=96 ! "
                f"queue ! udpsink host={config.laptopIPAddress} port={self.port}"
            )
        
            self.out = cv2.VideoWriter(
                streamConvert, cv2.CAP_GSTREAMER, 0,
                config.framesPerSecond, (config.frameWidth, config.frameHeight)
            ) 

            if self.out.isOpened():
                self.initSuccess = True
        else: 
            print(f"Cam {self.cameraID} failed to open") 

    def stream(self):
        try:
            if not self.initSuccess:
                return

            while True:
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    break
                self.out.write(frame)

        except Exception as e:
            print(f"Stream error on cam {self.cameraID}: {e}")
        finally:
            if self.cap:
                self.cap.release()
            if self.out:
                self.out.release()
            print(f"Cam {self.cameraID} resources released")
