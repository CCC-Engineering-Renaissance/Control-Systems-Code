import threading
import time
import sys #used to handle system level operations
import configCamera as config 
from camSend import cameraROV
#for future we may need to do peripherals.py not sure yet though...
def main():
    workers = []
    threads = []
    #loops through the cameras dictionary
    for cameraNames, info in config.cameras.items():
        worker = cameraROV(info["id"], info["port"])
        t = threading.Thread(target=worker.stream, daemon=True)
        t.start()

        workers.append(worker)
        threads.append(t)

        print(f"Started {cameraNames}: ID {info['id']} on Port {info['port']}")

    return workers, threads

if __name__ == "__main__":

    workers, threads = main() #starts the camera set ups from cameraROV first
    try:

        print("All cameras have been initialized. Press ctrl + c to stop")

        while True:

            time.sleep(1)
    #catches when you press Ctrl+C
    except KeyboardInterrupt:

        print("\nStopping all camera streams...")
        #exits the program
        sys.exit()
