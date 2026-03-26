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
import fcntl
import glob
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
# Configuration
# ---------------------------------------------------------------------------

STREAM_PORT   = 5000    # GUI connects here to receive frames
COMMAND_PORT  = 5001    # GUI connects here to send switch commands
JPEG_QUALITY  = 70      # 0-100; lower = smaller packets / higher latency tradeoff
FRAME_WIDTH   = 640
FRAME_HEIGHT  = 480
TARGET_FPS    = 15

# Camera slot names in the order they will be assigned to discovered devices.
# The first working /dev/videoN gets "front", the second gets "left", etc.
CAMERA_SLOTS = ["front", "left", "right", "bot", "back"]

# ---------------------------------------------------------------------------
# Auto-detect valid capture devices
# ---------------------------------------------------------------------------

# V4L2 ioctl constants (Linux kernel uapi/linux/videodev2.h)
# VIDIOC_QUERYCAP = _IOR('V', 0, struct v4l2_capability)  →  0x80685600
# struct v4l2_capability layout (104 bytes total):
#   driver[16]  card[32]  bus_info[32]  version[4]
#   capabilities[4]  device_caps[4]  reserved[12]
_VIDIOC_QUERYCAP        = 0x80685600
_V4L2_CAP_VIDEO_CAPTURE = 0x00000001   # this node can capture video frames
_V4L2_CAP_DEVICE_CAPS   = 0x80000000   # device_caps field is valid
_V4L2_CAP_SIZE          = 104
_CAPABILITIES_OFFSET    = 84   # physical-device capabilities (all nodes)
_DEVICE_CAPS_OFFSET     = 88   # per-node capabilities (this specific node)


def _is_v4l2_capture_device(dev_path: str) -> bool:
    """
    Return True only if this specific device node can capture video frames.

    VIDIOC_QUERYCAP exposes two capability fields:
      • capabilities  – OR of caps across all nodes of the physical device
      • device_caps   – caps for this specific /dev/videoN node only

    Checking only 'capabilities' (the old approach) causes false positives:
    metadata/output sibling nodes share the same physical device, so they
    inherit the VIDEO_CAPTURE bit even though they cannot capture frames.
    Using 'device_caps' (per-node) correctly excludes those siblings.
    """
    try:
        with open(dev_path, "rb") as f:
            buf = bytearray(_V4L2_CAP_SIZE)
            fcntl.ioctl(f, _VIDIOC_QUERYCAP, buf)
            caps        = struct.unpack_from("<I", buf, _CAPABILITIES_OFFSET)[0]
            device_caps = struct.unpack_from("<I", buf, _DEVICE_CAPS_OFFSET)[0]
            # Prefer per-node device_caps when the kernel supplies it
            effective = device_caps if (caps & _V4L2_CAP_DEVICE_CAPS) else caps
            return bool(effective & _V4L2_CAP_VIDEO_CAPTURE)
    except OSError:
        return False


def _probe_capture_devices() -> list[int]:
    """
    Scan /dev/video0 … /dev/video15 and return a sorted list of indices whose
    V4L2 capability flags include VIDEO_CAPTURE.

    Using the V4L2 ioctl instead of a test frame-read avoids false negatives
    caused by USB bandwidth contention when several cameras share a hub.
    """
    found = []
    candidates = sorted(
        int(p.replace("/dev/video", ""))
        for p in glob.glob("/dev/video[0-9]*")
    )
    for idx in candidates:
        dev_path = f"/dev/video{idx}"
        if _is_v4l2_capture_device(dev_path):
            found.append(idx)
            log.info("  /dev/video%d  ✓ capture device", idx)
        else:
            log.info("  /dev/video%d  ✗ not a capture device (skipped)", idx)
    return found


def _build_camera_map() -> dict[str, int]:
    """
    Pair each discovered capture device with a named camera slot.
    If fewer devices exist than slots, the extra slot names are omitted.
    If no devices exist at all, fall back to index 0.
    """
    devices = _probe_capture_devices()
    if not devices:
        log.warning("No capture devices found – falling back to index 0")
        devices = [0]

    camera_map = {}
    for slot, idx in zip(CAMERA_SLOTS, devices):
        camera_map[slot] = idx
        log.info("  Camera slot '%s'  →  /dev/video%d", slot, idx)

    log.info("Active camera map: %s", camera_map)
    return camera_map


# Built once at startup; used by CameraServer._open_camera
log.info("Probing video devices…")
CAMERA_MAP = _build_camera_map()

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
            log.warning(
                "Camera slot '%s' has no device assigned "
                "(only %d camera(s) detected at startup).",
                name, len(CAMERA_MAP),
            )
            return False

        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            log.error("Failed to open /dev/video%d for slot '%s'", idx, name)
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

            # Hold the lock for the entire read so that _open_camera cannot
            # call cap.release() on the same object while we are mid-read.
            # _open_camera also acquires this lock before releasing the old cap,
            # so the two operations are mutually exclusive.
            with self._lock:
                if self._cap is None:
                    time.sleep(0.1)
                    continue
                ret, frame = self._cap.read()

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
