import cv2
import configCamera as config 

class cameraROV:
    def __init__(self, cameraID, port):
        self.initSuccess = False
        self.cameraID = cameraID
        self.port = port

        self.cap = cv2.VideoCapture(self.cameraID, config.backend)

        for prop, value in config.propertyMap.items():
            self.cap.set(prop, value)

        ret, frame = self.cap.read()

        if self.cap.isOpened() and ret:
            #gstreamer pipeline for pi hardware encoding

            streamConvert = (
                f"appsrc ! videoconvert ! {config.encoder} ! "
                f"rtph264pay config-interval=1 pt=96 ! "
                f"queue ! udpsink host={config.laptopIPAddress} port={self.port}"
            )

            self.out = cv2.VideoWriter(
                streamConvert, cv2.CAP_GSTREAMER, 0, config.framesPerSecond, (config.frameWidth, config.frameHeight)
            )

            if self.out.isOpened():
                self.initSuccess = True 
                print(f"Cam {self.cameraID} ready on port {self.port}")

        else:
            print(f"Cam {self.cameraID} failed to open")

    def stream(self):
        try:
            if not self.initSuccess:
                return
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    break
                self.out.write(frame)
        except Exception as e:
            print(f"Error: {e}")
        finally:
            self.cap.release()
            if self.initSuccess:
                self.out.release()

