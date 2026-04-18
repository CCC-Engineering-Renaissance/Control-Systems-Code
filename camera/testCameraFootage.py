from flask import Flask, Response
import cv2
import configCamera as config

app = Flask(__name__)

# Try config.backend first, fall back to CAP_DSHOW if nothing opens
def try_open_cameras(backend):
    found = []
    for i in range(5):
        cap = cv2.VideoCapture(i, backend)
        if cap.isOpened():
            found.append((i, cap))
            print(f"Camera {i}: OK (backend={backend})")
        else:
            cap.release()
            print(f"Camera {i}: Not found (backend={backend})")
    return found

caps = try_open_cameras(config.backend)

if len(caps) == 0:
    print("No cameras found with config.backend, retrying with CAP_DSHOW...")
    caps = try_open_cameras(cv2.CAP_DSHOW)

if len(caps) == 0:
    print("No cameras found with CAP_DSHOW, retrying with CAP_MSMF...")
    caps = try_open_cameras(cv2.CAP_MSMF)

def generate_frames(index):
    cap = next((c for i, c in caps if i == index), None)
    if cap is None:
        return
    while True:
        success, frame = cap.read()
        if not success:
            break
        ret, buffer = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/cam<int:index>')
def video_feed(index):
    valid_indices = [i for i, _ in caps]
    if index not in valid_indices:
        return f"Camera {index} not available. Connected cameras: {valid_indices}", 404
    return Response(generate_frames(index), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    print(f"--- CAMERA TEST STARTING ({len(caps)} camera(s) found) ---")
    print("URLs: http://10.164.134.47:5000/cam0 through /cam4")
    app.run(host='0.0.0.0', port=5000, threaded=True)