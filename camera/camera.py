import threading
import time
import sys
import configCamera as config 
from cameraSender import cameraROV
#for future we may need to do peripherals.py not sure yet though...
def main():

    for cameraNames, info in config.cameras.items():

        #initializes the camera object

        cameraWorker = cameraROV(info["id"], info["port"])

        #starts the stream in its own thread 

        #daemon=True refers to the thread closing automatically once main script stops

        threading.Thread(target=cameraWorker.stream, daemon=True).start()

        print(f"Started {cameraNames}: ID {info['id']} on Port {info['port']}")

if __name__ == "__main__":

    main() #starts the camera set ups from cameraROV first
    try:

        print("All cameras have been initialized. Press ctrl + c to stop")

        while True:

            time.sleep(1)

    except KeyboardInterrupt:

        print("\nStopping all camera streams...")

        sys.exit()
