#!/usr/bin/env python3
"""
ROV Camera Stream Server  –  runs on the Raspberry Pi 5
=======================================================
Captures frames from one of multiple cameras and broadcasts them as
length-prefixed JPEG data over a TCP stream connection.
A second TCP port accepts camera-switch commands from the GUI.

Stream protocol  (Pi → GUI):
    [4-byte big-endian uint32: JPEG byte count][JPEG bytes]
    Repeats continuously for every frame captured.

Command protocol (GUI → Pi):
    "CMD:<name>\n"
    where <name> is one of: front, left, right, bot, back

Usage:
    python3 camera_server.py

Adjust CAMERA_MAP to match the physical camera wiring on your ROV.
For Raspberry Pi camera modules use index 0/1 (CSI) or /dev/video0 etc.
For USB cameras use the /dev/videoN index reported by `ls /dev/video*`.
"""

import cv2
import socket
import struct
import threading
import time
import logging

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Configuration – adjust to your ROV's physical camera wiring
# ---------------------------------------------------------------------------

# Maps camera name (sent by GUI buttons) to OpenCV VideoCapture index.
# If a camera is not physically present, the server falls back to index 0.
CAMERA_MAP = {
    "front": 0,
    "left":  1,
    "right": 2,
    "bot":   3,
    "back":  4,
}

STREAM_PORT   = 5000    # GUI connects here to receive frames
COMMAND_PORT  = 5001    # GUI connects here to send switch commands
JPEG_QUALITY  = 70      # 0-100; lower = smaller packets / higher latency tradeoff
FRAME_WIDTH   = 640
FRAME_HEIGHT  = 480
TARGET_FPS    = 15

# ---------------------------------------------------------------------------


class CameraServer:
    def __init__(self):
        self._lock         = threading.Lock()
        self._cap          = None           # active cv2.VideoCapture
        self._active_name  = "front"
        self._clients      = []             # connected stream sockets
        self._running      = False

    # ------------------------------------------------------------------
    # Camera management
    # ------------------------------------------------------------------

    def _open_camera(self, name: str) -> bool:
        idx = CAMERA_MAP.get(name)
        if idx is None:
            log.warning("Unknown camera '%s' – ignoring", name)
            return False

        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            log.warning(
                "Camera '%s' (index %d) not available – falling back to index 0",
                name, idx
            )
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                log.error("No camera available on this system")
                return False

        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS,          TARGET_FPS)

        with self._lock:
            if self._cap is not None:
                self._cap.release()
            self._cap         = cap
            self._active_name = name

        log.info("Camera set to '%s' (index %d)", name, idx)
        return True

    def switch_camera(self, name: str):
        """Thread-safe camera switch – runs in a background thread."""
        threading.Thread(
            target=self._open_camera, args=(name,), daemon=True
        ).start()

    # ------------------------------------------------------------------
    # Frame broadcasting
    # ------------------------------------------------------------------

    def _broadcast(self, jpeg_bytes: bytes):
        """Prepend 4-byte length header and send to all stream clients."""
        payload = struct.pack(">I", len(jpeg_bytes)) + jpeg_bytes

        dead = []
        with self._lock:
            clients = list(self._clients)

        for sock in clients:
            try:
                sock.sendall(payload)
            except OSError:
                dead.append(sock)

        if dead:
            with self._lock:
                for s in dead:
                    try:
                        self._clients.remove(s)
                        s.close()
                    except (ValueError, OSError):
                        pass

    # ------------------------------------------------------------------
    # Capture loop
    # ------------------------------------------------------------------

    def _capture_loop(self):
        interval      = 1.0 / TARGET_FPS
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]

        # Wait until a camera is ready
        while self._running:
            with self._lock:
                cap = self._cap
            if cap is not None:
                break
            time.sleep(0.1)

        log.info("Capture loop started at %d FPS", TARGET_FPS)

        while self._running:
            t0 = time.monotonic()

            with self._lock:
                cap = self._cap

            if cap is None:
                time.sleep(0.1)
                continue

            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            ok, buf = cv2.imencode(".jpg", frame, encode_params)
            if ok:
                self._broadcast(buf.tobytes())

            elapsed = time.monotonic() - t0
            wait    = interval - elapsed
            if wait > 0:
                time.sleep(wait)

    # ------------------------------------------------------------------
    # Stream TCP server  (clients receive raw frames here)
    # ------------------------------------------------------------------

    def _stream_listener(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("", STREAM_PORT))
        srv.listen(5)
        srv.settimeout(1.0)
        log.info("Stream server listening on TCP port %d", STREAM_PORT)

        while self._running:
            try:
                conn, addr = srv.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                log.info("Stream client connected: %s", addr)
                with self._lock:
                    self._clients.append(conn)
            except socket.timeout:
                pass
            except OSError:
                break

        srv.close()

    # ------------------------------------------------------------------
    # Command TCP server  (clients send camera switch commands here)
    # ------------------------------------------------------------------

    def _handle_command_client(self, conn: socket.socket, addr):
        log.info("Command client connected: %s", addr)
        buf = b""
        try:
            while self._running:
                data = conn.recv(64)
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    cmd = line.decode("ascii", errors="ignore").strip()
                    if cmd.startswith("CMD:"):
                        cam_name = cmd[4:].lower().strip()
                        log.info("Switch command received: '%s'", cam_name)
                        self.switch_camera(cam_name)
        except OSError:
            pass
        finally:
            conn.close()
            log.info("Command client disconnected: %s", addr)

    def _command_listener(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("", COMMAND_PORT))
        srv.listen(5)
        srv.settimeout(1.0)
        log.info("Command server listening on TCP port %d", COMMAND_PORT)

        while self._running:
            try:
                conn, addr = srv.accept()
                threading.Thread(
                    target=self._handle_command_client,
                    args=(conn, addr),
                    daemon=True,
                ).start()
            except socket.timeout:
                pass
            except OSError:
                break

        srv.close()

    # ------------------------------------------------------------------
    # Public start / stop
    # ------------------------------------------------------------------

    def start(self):
        self._running = True
        self._open_camera("front")  # open default camera synchronously
        for target in (
            self._stream_listener,
            self._command_listener,
            self._capture_loop,
        ):
            threading.Thread(target=target, daemon=True).start()
        log.info(
            "Camera server started  |  stream=%d  commands=%d  fps=%d",
            STREAM_PORT, COMMAND_PORT, TARGET_FPS,
        )

    def stop(self):
        self._running = False
        log.info("Camera server stopped")


# ---------------------------------------------------------------------------

if __name__ == "__main__":
    server = CameraServer()
    server.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        server.stop()
        log.info("Goodbye")
