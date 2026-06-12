#!/usr/bin/env python3
"""
pi_launcher.py – ROV task launcher daemon (runs on the Raspberry Pi)
====================================================================
Lets the topside GUI start/stop the Pi-side programs without SSH.
Stdlib only — nothing to pip-install.

Protocol: newline-delimited JSON over TCP (default port 5010).
Every reply echoes "cmd" (and "task" where relevant) plus "ok".

  {"cmd":"ping"}                          -> {"cmd":"ping","ok":true,"version":1}
  {"cmd":"status"}                        -> {"cmd":"status","ok":true,
                                              "tasks":{"oneservo":{"state":"running","pid":123},
                                                       "camera":{"state":"exited","code":0},
                                                       "build":{"state":"idle"}}}
  {"cmd":"start","task":"camera"}         -> {"cmd":"start","task":"camera","ok":true}
  {"cmd":"stop","task":"camera"}          -> {"cmd":"stop","task":"camera","ok":true}
  {"cmd":"tail","task":"camera","since":N}-> {"cmd":"tail","task":"camera","ok":true,
                                              "lines":[...],"next":M}

States: "idle" (never started), "running", "exited" (+ "code").
stop sends SIGINT (clean Ctrl+C path), then SIGTERM after 2 s, SIGKILL after 5 s.

Run once on the Pi via systemd: see rov-launcher.service / install_pi.sh.
"""

from __future__ import annotations

import argparse
import collections
import json
import os
import signal
import socket
import subprocess
import threading
import time

PROTOCOL_VERSION = 1
TAIL_KEEP = 500          # output lines kept per task

# name -> (command, cwd relative to --dir, pre-start check relative to --dir)
TASKS: dict[str, dict] = {
    "oneservo": {
        "cmd": ["./OneServo"],
        "cwd": "MAIN_CODE",
        "requires": "MAIN_CODE/OneServo",
        "missing_msg": "OneServo binary not found — run build first",
    },
    "camera": {
        "cmd": ["python3", "-u", "camera_server.py"],
        "cwd": ".",
        "requires": "camera_server.py",
        "missing_msg": "camera_server.py not found in --dir",
    },
    "build": {
        "cmd": ["make", "OneServo"],
        "cwd": "MAIN_CODE",
        "requires": "MAIN_CODE/Makefile",
        "missing_msg": "MAIN_CODE/Makefile not found in --dir",
    },
}


class TaskProc:
    """One managed subprocess: state, ring buffer of output lines."""

    def __init__(self, name: str, spec: dict, base_dir: str) -> None:
        self.name = name
        self.spec = spec
        self.base_dir = base_dir
        self.lock = threading.Lock()
        self.proc: subprocess.Popen | None = None
        self.lines: collections.deque[tuple[int, str]] = collections.deque(maxlen=TAIL_KEEP)
        self.seq = 0

    # -- helpers ------------------------------------------------------------

    def _append_line(self, line: str) -> None:
        with self.lock:
            self.seq += 1
            self.lines.append((self.seq, line))

    def _reader(self, proc: subprocess.Popen) -> None:
        assert proc.stdout is not None
        for raw in proc.stdout:
            self._append_line(raw.rstrip("\r\n"))
        proc.stdout.close()

    # -- commands -----------------------------------------------------------

    def status(self) -> dict:
        with self.lock:
            proc = self.proc
        if proc is None:
            return {"state": "idle"}
        code = proc.poll()
        if code is None:
            return {"state": "running", "pid": proc.pid}
        return {"state": "exited", "code": code}

    def start(self) -> tuple[bool, str]:
        with self.lock:
            if self.proc is not None and self.proc.poll() is None:
                return False, "already running"
        required = os.path.join(self.base_dir, self.spec["requires"])
        if not os.path.exists(required):
            return False, self.spec["missing_msg"]

        cwd = os.path.join(self.base_dir, self.spec["cwd"])
        try:
            proc = subprocess.Popen(
                self.spec["cmd"],
                cwd=cwd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                text=True,
                bufsize=1,
                # The daemon may run with SIGINT ignored (e.g. launched in the
                # background); children must still die to our "stop" SIGINT.
                preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL),
            )
        except OSError as exc:
            return False, f"failed to start: {exc}"

        with self.lock:
            self.proc = proc
        self._append_line(f"--- started: {' '.join(self.spec['cmd'])} (pid {proc.pid}) ---")
        threading.Thread(target=self._reader, args=(proc,), daemon=True).start()
        return True, ""

    def stop(self) -> tuple[bool, str]:
        with self.lock:
            proc = self.proc
        if proc is None or proc.poll() is not None:
            return False, "not running"

        def escalate() -> None:
            try:
                proc.send_signal(signal.SIGINT)
                time.sleep(2.0)
                if proc.poll() is None:
                    proc.terminate()
                    time.sleep(3.0)
                if proc.poll() is None:
                    proc.kill()
            except OSError:
                pass

        threading.Thread(target=escalate, daemon=True).start()
        return True, ""

    def tail(self, since: int) -> tuple[list[str], int]:
        with self.lock:
            out = [line for seq, line in self.lines if seq > since]
            return out, self.seq


def handle_request(req: dict, tasks: dict[str, TaskProc]) -> dict:
    cmd = req.get("cmd")
    if cmd == "ping":
        return {"cmd": "ping", "ok": True, "version": PROTOCOL_VERSION}

    if cmd == "status":
        return {"cmd": "status", "ok": True,
                "tasks": {name: tp.status() for name, tp in tasks.items()}}

    if cmd in ("start", "stop", "tail"):
        name = req.get("task")
        tp = tasks.get(name)
        if tp is None:
            return {"cmd": cmd, "task": name, "ok": False, "error": "unknown task"}
        if cmd == "start":
            ok, err = tp.start()
        elif cmd == "stop":
            ok, err = tp.stop()
        else:
            lines, nxt = tp.tail(int(req.get("since", 0)))
            return {"cmd": "tail", "task": name, "ok": True,
                    "lines": lines, "next": nxt}
        reply = {"cmd": cmd, "task": name, "ok": ok}
        if not ok:
            reply["error"] = err
        return reply

    return {"cmd": cmd, "ok": False, "error": "bad request"}


def handle_connection(conn: socket.socket, addr, tasks: dict[str, TaskProc]) -> None:
    print(f"client connected: {addr[0]}:{addr[1]}")
    try:
        with conn, conn.makefile("r", encoding="utf-8", errors="replace") as rx:
            for raw in rx:
                raw = raw.strip()
                if not raw:
                    continue
                try:
                    req = json.loads(raw)
                    if not isinstance(req, dict):
                        raise ValueError
                except ValueError:
                    reply = {"ok": False, "error": "bad request"}
                else:
                    reply = handle_request(req, tasks)
                conn.sendall((json.dumps(reply) + "\n").encode("utf-8"))
    except OSError:
        pass
    print(f"client disconnected: {addr[0]}:{addr[1]}")


def main() -> None:
    parser = argparse.ArgumentParser(description="ROV task launcher daemon")
    parser.add_argument("--port", type=int, default=5010,
                        help="TCP port to listen on (default 5010)")
    parser.add_argument("--dir", default=os.path.dirname(os.path.abspath(__file__)),
                        help="Control-Systems-Code directory (default: script's own)")
    args = parser.parse_args()

    base_dir = os.path.abspath(os.path.expanduser(args.dir))
    tasks = {name: TaskProc(name, spec, base_dir) for name, spec in TASKS.items()}

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", args.port))
    server.listen(4)
    print(f"pi_launcher listening on 0.0.0.0:{args.port}, dir={base_dir}")
    print(f"tasks: {', '.join(TASKS)}")

    try:
        while True:
            conn, addr = server.accept()
            threading.Thread(target=handle_connection, args=(conn, addr, tasks),
                             daemon=True).start()
    except KeyboardInterrupt:
        print("\npi_launcher stopped.")
    finally:
        server.close()


if __name__ == "__main__":
    main()
