import cv2
import socket
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
target = ("192.168.8.161", 5005)
target2 = ("192.168.8.161", 5006)
target3 = ("192.168.8.161", 5007)
target4 = ("192.168.8.161", 5008)
target5 = ("192.168.8.161", 5009)

desired_width = 1920  # Example width
desired_height = 1080  # Example height
fps_limit = 15
frame_time = 1 / fps_limit

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))

cap1 = cv2.VideoCapture(4, cv2.CAP_V4L2)
cap1.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)
cap1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))

cap2 = cv2.VideoCapture(8, cv2.CAP_V4L2)
cap2.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)
cap2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))

cap3 = cv2.VideoCapture(16, cv2.CAP_V4L2)
cap3.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
cap3.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)
cap3.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))

cap4 = cv2.VideoCapture(32, cv2.CAP_V4L2)
cap4.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
cap4.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)
cap4.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))

while True:
    start = time.time()

    ret, frame = cap.read()
    if ret:
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        _, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        sock.sendto(buffer.tobytes(), target)

    ret, frame = cap1.read()
    if ret:
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        _, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        sock.sendto(buffer.tobytes(), target2)

    ret, frame = cap2.read()
    if ret:
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        _, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        sock.sendto(buffer.tobytes(), target3)

    ret, frame = cap3.read()
    if ret:
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        _, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        sock.sendto(buffer.tobytes(), target4)

    ret, frame = cap4.read()
    if ret:
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        _, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        sock.sendto(buffer.tobytes(), target5)

    elapsed = time.time() - start
    if elapsed < frame_time:
        time.sleep(frame_time - elapsed)
