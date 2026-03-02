import multiprocessing
import time 
import sys
import configCamera as config
from camSend import cameraROV 
 
def startCamera(name, cameraID, port):
    worker = cameraROV(cameraID, port)
    if worker.initSuccess:
        print(f"Process Started: {name} (ID {cameraID} on Port {port})")
        worker.stream()
    else:
        print(f"Process Failed: {name}")

def main(): 
    cameraProcesses = []
    for cameraName, info in config.cameras.items():
        p = multiprocessing.Process(
            target = startCamera,
            args = (cameraName, info["id"], info["port"]),
            daemon = True
        )
        p.start()
        cameraProcesses.append(p)
    return cameraProcesses

if __name__ == "__main__": 
    activeStreams = main()
    try: 
        print("ROV Camera System Active. Press Ctrl+C to stop.")
        while True: 
            time.sleep(1) 
    except KeyboardInterrupt: 
        print("\nStopping all camera processes...")
        for p in activeStreams:
            p.terminate()
            p.join()
        print("Exiting.")
        sys.exit() 





 
