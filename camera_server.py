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

import os
# Silence OpenCV's own verbose error output before importing cv2.
# Without this, a disconnected USB camera floods the terminal with
# repeated "Not a video capture device" lines from every failed ioctl.
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"

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

STREAM_PORT  = 5000   # GUI connects here to receive frames
COMMAND_PORT = 5001   # GUI connects here to send switch commands

# Capture modes matching the cameras' native MJPEG modes (two camera types).
#   "lo"   – 1280×720  @ 30fps  – low-bandwidth / lowest USB power draw
#   "live" – 1920×1080 @ 30fps  – 1080p cameras; normal ROV piloting
#   "hi"   – 2048×1536 @ 30fps  –  3 MP cameras at full resolution
#   "hq"   – 4656×3496 @ 10fps  – 16 MP camera; photogrammetry capture
#
# The camera outputs MJPEG natively at these modes, so USB bandwidth stays
# manageable even at 16 MP.  JPEG_QUALITY only affects the re-encode done
# by the Pi before sending over Ethernet (not the USB capture itself).
MODES = {
    "lo":   {"width": 1280, "height": 720,  "fps": 30, "jpeg_quality": 70},
    "live": {"width": 1920, "height": 1080, "fps": 30, "jpeg_quality": 70},
    "hi":   {"width": 2048, "height": 1536, "fps": 30, "jpeg_quality": 70},
    "hq":   {"width": 4656, "height": 3496, "fps": 10, "jpeg_quality": 92},
}
DEFAULT_MODE = "live"

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
_BUS_INFO_OFFSET        = 48   # bus_info[32] — e.g. b"usb-..." for USB cameras
_CAPABILITIES_OFFSET    = 84   # physical-device capabilities (all nodes)
_DEVICE_CAPS_OFFSET     = 88   # per-node capabilities (this specific node)


def _is_v4l2_capture_device(dev_path: str) -> bool:
    """
    Return True only if this node is a *USB* camera that can capture frames.

    Two checks, both from VIDIOC_QUERYCAP:

    1. Per-node VIDEO_CAPTURE. device_caps is the per-node capability set;
       'capabilities' is OR'd across all nodes of the physical device, so
       metadata/output siblings inherit the VIDEO_CAPTURE bit they can't honor.
       Using device_caps excludes those siblings.

    2. bus_info starts with "usb-". On the Pi 5 (RP1) the internal CFE / ISP /
       codec nodes (e.g. /dev/video0-3, video19+) ALSO advertise VIDEO_CAPTURE,
       but they can't be streamed like a webcam — opening them fails with
       "Not a video capture device", which is exactly what stalled the feed.
       Those nodes have a "platform:" bus_info; requiring "usb-" skips them and
       reliably selects the real USB camera(s) regardless of index or
       enumeration order.  (Note: this intentionally ignores CSI/Pi-camera-
       module inputs — this ROV uses USB cameras.)
    """
    try:
        with open(dev_path, "rb") as f:
            buf = bytearray(_V4L2_CAP_SIZE)
            fcntl.ioctl(f, _VIDIOC_QUERYCAP, buf)
            caps        = struct.unpack_from("<I", buf, _CAPABILITIES_OFFSET)[0]
            device_caps = struct.unpack_from("<I", buf, _DEVICE_CAPS_OFFSET)[0]
            # Prefer per-node device_caps when the kernel supplies it
            effective = device_caps if (caps & _V4L2_CAP_DEVICE_CAPS) else caps
            if not (effective & _V4L2_CAP_VIDEO_CAPTURE):
                return False
            bus_info = bytes(
                buf[_BUS_INFO_OFFSET:_BUS_INFO_OFFSET + 32]
            ).split(b"\x00", 1)[0]
            return bus_info.startswith(b"usb-")
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
        self._clients      = {}             # stream socket -> outgoing bytearray
        self._running      = False
        self._mode         = DEFAULT_MODE   # "live" or "hq", switchable at runtime

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

        # Retry up to 3 times — a USB camera on a hub can take a moment to
        # become available again after a brief disconnect or power fluctuation.
        for attempt in range(1, 4):
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                break
            cap.release()
            if attempt < 3:
                log.warning(
                    "Cannot open /dev/video%d for slot '%s' (attempt %d/3) – retrying…",
                    idx, name, attempt,
                )
                time.sleep(1.0)
        else:
            log.error(
                "Failed to open /dev/video%d for slot '%s' after 3 attempts. "
                "Camera may be physically disconnected.",
                idx, name,
            )
            return False

        with self._lock:
            mode = MODES[self._mode]

        # Request MJPEG from the camera so the sensor compresses on-chip.
        # This keeps USB bandwidth low even at 16 MP.
        cap.set(cv2.CAP_PROP_FOURCC,       cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  mode["width"])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, mode["height"])
        cap.set(cv2.CAP_PROP_FPS,          mode["fps"])

        # Forward the camera's MJPEG frames without decoding them: ask OpenCV to
        # return the raw JPEG bytes (CONVERT_RGB=0) instead of a decoded BGR
        # image, and keep only the newest frame. This drops a full JPEG
        # decode + re-encode per frame on the Pi — the main FPS bottleneck.
        # If a camera/backend ignores CONVERT_RGB, read() still returns a
        # 3-D BGR array and the capture loop falls back to re-encoding.
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        cap.set(cv2.CAP_PROP_BUFFERSIZE,  1)

        # Log what the camera ACTUALLY negotiated vs what we asked for — if the
        # driver caps fps at this resolution it shows up here (e.g. "@ 15.0fps"
        # when we requested 30), which tells us the limit is the sensor/USB and
        # not the GUI or network.
        fourcc_int = int(cap.get(cv2.CAP_PROP_FOURCC))
        fourcc = "".join(chr((fourcc_int >> (8 * i)) & 0xFF) for i in range(4))
        log.info(
            "Negotiated: %s %dx%d @ %.1ffps  (requested %dx%d @ %dfps)",
            fourcc,
            int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            cap.get(cv2.CAP_PROP_FPS),
            mode["width"], mode["height"], mode["fps"],
        )

        with self._lock:
            if self._cap is not None:
                self._cap.release()
            self._cap         = cap
            self._active_name = name

        log.info(
            "Camera set to '%s' (index %d)  mode=%s  %dx%d @ %dfps",
            name, idx, self._mode, mode["width"], mode["height"], mode["fps"],
        )
        return True

    def switch_camera(self, name: str):
        """Thread-safe camera switch – runs in a background thread."""
        threading.Thread(
            target=self._open_camera, args=(name,), daemon=True
        ).start()

    def set_mode(self, mode: str):
        """Switch capture resolution/fps mode and reopen the active camera."""
        if mode not in MODES:
            log.warning("Unknown mode '%s' (must be one of %s)", mode, list(MODES))
            return
        with self._lock:
            self._mode        = mode
            active_name       = self._active_name
        cfg = MODES[mode]
        log.info(
            "Mode set to '%s'  →  %dx%d @ %dfps",
            mode, cfg["width"], cfg["height"], cfg["fps"],
        )
        # Reopen the current camera at the new resolution
        threading.Thread(
            target=self._open_camera, args=(active_name,), daemon=True
        ).start()

    # ------------------------------------------------------------------
    # Frame broadcasting
    # ------------------------------------------------------------------

    def _broadcast(self, jpeg_bytes: bytes):
        """Queue a framed JPEG to every stream client and flush without blocking.

        Each client has its own outgoing buffer. A client that has fully drained
        its buffer receives the new frame; a client still behind has this frame
        dropped (keep-latest). All sends are non-blocking, so one slow client
        (e.g. on weak Wi-Fi) can never stall the capture loop for the others —
        which previously dragged the whole stream down to a few fps.
        """
        payload = struct.pack(">I", len(jpeg_bytes)) + jpeg_bytes

        with self._lock:
            clients = list(self._clients.items())

        dead = []
        for sock, pending in clients:
            if not pending:
                pending.extend(payload)        # caught up -> send newest frame
            # else: client is behind -> drop this frame for it (keep-latest)
            try:
                while pending:
                    sent = sock.send(pending)
                    if sent <= 0:
                        break
                    del pending[:sent]
            except BlockingIOError:
                pass                           # socket full; keep remainder for next call
            except OSError:
                dead.append(sock)

        if dead:
            with self._lock:
                for s in dead:
                    self._clients.pop(s, None)
                    try:
                        s.close()
                    except OSError:
                        pass

    # ------------------------------------------------------------------
    # Capture loop
    # ------------------------------------------------------------------

    def _capture_loop(self):

        # Wait until a camera is ready
        while self._running:
            with self._lock:
                ready = self._cap is not None
            if ready:
                break
            time.sleep(0.1)

        log.info("Capture loop started")

        consecutive_failures = 0
        FAILURE_WARN_THRESHOLD = 10   # log a warning after this many bad reads
        FAILURE_DROP_THRESHOLD = 30   # give up and clear cap after this many

        while self._running:
            t0 = time.monotonic()

            # Hold the lock for the entire read so that _open_camera cannot
            # call cap.release() on the same object while we are mid-read.
            with self._lock:
                if self._cap is None:
                    time.sleep(0.1)
                    consecutive_failures = 0
                    continue
                ret, frame = self._cap.read()
                active_name = self._active_name

            if not ret:
                consecutive_failures += 1
                if consecutive_failures == FAILURE_WARN_THRESHOLD:
                    log.warning(
                        "Camera '%s' (/dev/video%d): repeated read failures – "
                        "possible USB disconnect or bandwidth issue.",
                        active_name, CAMERA_MAP.get(active_name, -1),
                    )
                if consecutive_failures >= FAILURE_DROP_THRESHOLD:
                    log.error(
                        "Camera '%s' appears disconnected after %d failures. "
                        "Clearing active camera – switch to another view.",
                        active_name, consecutive_failures,
                    )
                    with self._lock:
                        if self._cap is not None:
                            self._cap.release()
                            self._cap = None
                    consecutive_failures = 0
                time.sleep(0.05)
                continue

            consecutive_failures = 0

            with self._lock:
                mode_cfg = MODES[self._mode]

            # With CONVERT_RGB=0 the camera's MJPEG comes back as raw bytes (a
            # 1-D / single-row array) — broadcast it untouched. A 3-D array means
            # OpenCV decoded to BGR after all, so re-encode that frame to JPEG.
            if frame.ndim == 3:
                ok, buf = cv2.imencode(
                    ".jpg", frame,
                    [cv2.IMWRITE_JPEG_QUALITY, mode_cfg["jpeg_quality"]],
                )
                data = buf.tobytes() if ok else None
            else:
                data = frame.tobytes()

            if data:
                self._broadcast(data)

            elapsed = time.monotonic() - t0
            wait    = (1.0 / mode_cfg["fps"]) - elapsed
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
                conn.setblocking(False)
                log.info("Stream client connected: %s", addr)
                with self._lock:
                    self._clients[conn] = bytearray()
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
                    elif cmd.startswith("MODE:"):
                        mode_name = cmd[5:].lower().strip()
                        log.info("Mode command received: '%s'", mode_name)
                        self.set_mode(mode_name)
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
            "Camera server started  |  stream=%d  commands=%d  default_mode=%s",
            STREAM_PORT, COMMAND_PORT, DEFAULT_MODE,
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
