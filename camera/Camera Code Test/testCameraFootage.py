from flask import Flask, Response
import cv2
import configCamera as config

app = Flask(__name__)

# Create a list of 5 camera objects (IDs 0, 1, 2, 3, 4)
caps = []
for i in range(5):
    caps.append(cv2.VideoCapture(i, config.backend))

def generate_frames(index):
    camera = caps[index]
    while True:
        success, frame = camera.read()
        if not success:
            # If camera isn't plugged in, break or show a "No Signal" frame
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# Create 5 routes dynamically
@app.route('/cam0')
def cam0(): return Response(generate_frames(0), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/cam1')
def cam1(): return Response(generate_frames(1), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/cam2')
def cam2(): return Response(generate_frames(2), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/cam3')
def cam3(): return Response(generate_frames(3), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/cam4')
def cam4(): return Response(generate_frames(4), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    print("--- 5-CAMERA TEST STARTING ---")
    print("URLs: http://127.0.0.1:5000/cam0 through /cam4")
    app.run(host='0.0.0.0', port=5000, threaded=True)
