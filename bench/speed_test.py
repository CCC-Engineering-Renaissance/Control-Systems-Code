"""
bench/speed_test.py  –  UDP Data Transfer & Speed Test
=======================================================
Measures the real network-stack performance of the ROV control link
using a loopback UDP echo server.

Four test suites
----------------
1. Loopback RTT      – min / avg / p95 / max round-trip time over lo
2. Send-rate accuracy – how close to the target Hz does the timer run?
   Tested at 30 Hz, 50 Hz, 100 Hz, 200 Hz
3. Burst throughput  – max packets/sec and KB/s (fire-and-forget)
4. Jitter analysis   – std-dev and histogram of inter-send intervals

Usage
-----
    source venv/bin/activate
    python bench/speed_test.py           # full suite
    python bench/speed_test.py --rtt     # RTT only
    python bench/speed_test.py --rate    # send-rate accuracy only
    python bench/speed_test.py --burst   # burst throughput only
    python bench/speed_test.py --jitter  # jitter analysis only
"""

import argparse
import math
import os
import socket
import statistics
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

# ── Headless pygame mock ─────────────────────────────────────────────────────
import types as _types
_fp = _types.ModuleType("pygame");     _fp.init = _fp.quit = lambda: None
_fj = _types.ModuleType("pygame.joystick"); _fj.init = lambda: None; _fj.get_count = lambda: 0
class _FJ: pass
_fj.Joystick = _FJ; _fp.joystick = _fj
_fd = _types.ModuleType("pygame.display")
_fd.set_mode = _fd.set_caption = lambda *a, **kw: None; _fp.display = _fd
_fp.QUIT = 256; _fp.KEYDOWN = 768; _fp.K_ESCAPE = 27
_fe = _types.ModuleType("pygame.event"); _fe.pump = lambda: None; _fe.get = lambda: []
_fp.event = _fe
for _k, _v in {"pygame": _fp, "pygame.joystick": _fj,
               "pygame.display": _fd, "pygame.event": _fe}.items():
    sys.modules.setdefault(_k, _v)

from thruster import _build_packet  # noqa: E402

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

LOOPBACK = "127.0.0.1"

def _free_port() -> int:
    """Return an available UDP port."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("", 0))
        return s.getsockname()[1]


def _hdr(title: str) -> None:
    print(f"\n{'═'*62}")
    print(f"  {title}")
    print(f"{'═'*62}")


def _row(label: str, value: str) -> None:
    print(f"  {label:<38} {value}")


# Typical in-flight packet (mid-range axes)
_SAMPLE_PKT = _build_packet(0.45, -0.20, 0.30, 0.10, -0.15, 0.0, 1, 0, 0.6)


# ---------------------------------------------------------------------------
# 1. Loopback RTT
# ---------------------------------------------------------------------------

class _EchoServer:
    """Tiny UDP echo server: reflects every datagram back to the sender."""

    def __init__(self):
        self.port = _free_port()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((LOOPBACK, self.port))
        self._running = False

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        self._sock.close()

    def _run(self):
        while self._running:
            try:
                data, addr = self._sock.recvfrom(4096)
                self._sock.sendto(data, addr)
            except OSError:
                break


def measure_rtt(n: int = 500, timeout_s: float = 0.5) -> dict:
    """Send n ping packets to a loopback echo server and measure RTT."""
    server = _EchoServer()
    server.start()

    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.settimeout(timeout_s)

    # Ping packet: 8-byte big-endian timestamp (ns)
    rtts = []
    dropped = 0

    for _ in range(n):
        ts = time.perf_counter_ns()
        ping = struct.pack(">Q", ts)
        client.sendto(ping, (LOOPBACK, server.port))
        try:
            pong, _ = client.recvfrom(64)
            rtt_us = (time.perf_counter_ns() - ts) / 1_000
            rtts.append(rtt_us)
        except socket.timeout:
            dropped += 1

    client.close()
    server.stop()

    if not rtts:
        return {"error": "all packets dropped"}

    rtts.sort()
    return {
        "n":          n,
        "dropped":    dropped,
        "loss_pct":   dropped / n * 100,
        "min_us":     rtts[0],
        "avg_us":     statistics.mean(rtts),
        "p50_us":     rtts[len(rtts) // 2],
        "p95_us":     rtts[int(len(rtts) * 0.95)],
        "max_us":     rtts[-1],
        "stddev_us":  statistics.stdev(rtts) if len(rtts) > 1 else 0.0,
    }


def test_rtt() -> None:
    _hdr("1. Loopback Round-Trip Time (RTT)")
    r = measure_rtt(n=500)
    if "error" in r:
        print(f"  ERROR: {r['error']}")
        return
    _row("Packets sent / received",
         f"{r['n']} / {r['n'] - r['dropped']}  "
         f"(loss {r['loss_pct']:.1f} %)")
    _row("RTT  min",          f"{r['min_us']:.1f} µs")
    _row("RTT  avg",          f"{r['avg_us']:.1f} µs")
    _row("RTT  p50 (median)", f"{r['p50_us']:.1f} µs")
    _row("RTT  p95",          f"{r['p95_us']:.1f} µs")
    _row("RTT  max",          f"{r['max_us']:.1f} µs")
    _row("RTT  std-dev",      f"{r['stddev_us']:.1f} µs")


# ---------------------------------------------------------------------------
# 2. Send-rate accuracy
# ---------------------------------------------------------------------------

def measure_send_rate(target_hz: int, duration_s: float = 3.0) -> dict:
    """Send ROV packets at target_hz for duration_s seconds.

    Returns achieved Hz, timing std-dev, and min/max inter-packet gap.
    """
    port   = _free_port()
    server = _EchoServer()
    server.start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / target_hz

    intervals = []
    t_start   = time.perf_counter()
    t_last    = t_start
    sent      = 0

    while time.perf_counter() - t_start < duration_s:
        now = time.perf_counter()
        if now - t_last >= period:
            sock.sendto(_SAMPLE_PKT, (LOOPBACK, server.port))
            intervals.append(now - t_last)
            t_last = now
            sent  += 1
        # busy-wait for sub-millisecond timing (same pattern as main loop)
        time.sleep(0.0001)

    sock.close()
    server.stop()

    elapsed   = time.perf_counter() - t_start
    achieved  = sent / elapsed
    intervals = intervals[1:]  # skip first (includes startup delay)
    ideal_ms  = 1000.0 / target_hz

    return {
        "target_hz":   target_hz,
        "sent":        sent,
        "elapsed_s":   elapsed,
        "achieved_hz": achieved,
        "hz_error_pct": abs(achieved - target_hz) / target_hz * 100,
        "interval_ideal_ms": ideal_ms,
        "interval_avg_ms":   statistics.mean(intervals) * 1000 if intervals else 0,
        "interval_min_ms":   min(intervals) * 1000 if intervals else 0,
        "interval_max_ms":   max(intervals) * 1000 if intervals else 0,
        "interval_std_ms":   statistics.stdev(intervals) * 1000 if len(intervals) > 1 else 0,
    }


def test_send_rate() -> None:
    _hdr("2. Send-Rate Accuracy")
    print(f"  {'Hz':>6}  {'Achieved':>10}  {'Error':>8}  "
          f"{'Ideal ms':>10}  {'Avg ms':>8}  {'Std ms':>8}  {'Max ms':>8}")
    print(f"  {'─'*6}  {'─'*10}  {'─'*8}  {'─'*10}  {'─'*8}  {'─'*8}  {'─'*8}")

    for hz in (30, 50, 100, 200):
        r = measure_send_rate(hz, duration_s=2.0)
        print(
            f"  {r['target_hz']:>6}  "
            f"{r['achieved_hz']:>9.2f}  "
            f"{r['hz_error_pct']:>7.2f}%  "
            f"{r['interval_ideal_ms']:>10.3f}  "
            f"{r['interval_avg_ms']:>8.3f}  "
            f"{r['interval_std_ms']:>8.3f}  "
            f"{r['interval_max_ms']:>8.3f}"
        )


# ---------------------------------------------------------------------------
# 3. Burst throughput
# ---------------------------------------------------------------------------

def measure_burst(n: int = 50_000) -> dict:
    """Fire-and-forget: send n packets as fast as possible."""
    port = _free_port()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (LOOPBACK, port)

    t0 = time.perf_counter()
    for _ in range(n):
        sock.sendto(_SAMPLE_PKT, target)
    elapsed = time.perf_counter() - t0

    sock.close()
    pkt_size  = len(_SAMPLE_PKT)
    pps       = n / elapsed
    kbps      = pkt_size * pps / 1024

    return {
        "n":          n,
        "elapsed_s":  elapsed,
        "pps":        pps,
        "kbps":       kbps,
        "mbps":       kbps / 1024,
        "pkt_bytes":  pkt_size,
        "us_per_pkt": elapsed / n * 1e6,
    }


def test_burst() -> None:
    _hdr("3. Burst Throughput (fire-and-forget, loopback)")
    r = measure_burst(n=50_000)
    _row("Packets sent",          f"{r['n']:,}")
    _row("Elapsed",               f"{r['elapsed_s']*1000:.1f} ms")
    _row("Packet size",           f"{r['pkt_bytes']} bytes")
    _row("Throughput",            f"{r['pps']:,.0f} pkt/s  →  {r['kbps']:.0f} KB/s  "
                                  f"({r['mbps']:.2f} MB/s)")
    _row("Time per packet",       f"{r['us_per_pkt']:.2f} µs")
    _row("30 Hz budget used",     f"{30 / r['pps'] * 100:.4f} % of 33.3 ms frame")
    _row("30 Hz bandwidth used",  f"{r['pkt_bytes'] * 30 / 1024:.2f} KB/s  "
                                  f"(of {r['kbps']:.0f} KB/s available)")


# ---------------------------------------------------------------------------
# 4. Jitter analysis
# ---------------------------------------------------------------------------

def measure_jitter(target_hz: int = 30, duration_s: float = 5.0) -> dict:
    """Collect inter-packet send timestamps and compute jitter statistics."""
    sock    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    port    = _free_port()
    target  = (LOOPBACK, port)
    period  = 1.0 / target_hz
    t_start = time.perf_counter()
    t_last  = t_start
    stamps  = []

    while time.perf_counter() - t_start < duration_s:
        now = time.perf_counter()
        if now - t_last >= period:
            sock.sendto(_SAMPLE_PKT, target)
            stamps.append(now)
            t_last = now
        time.sleep(0.0001)

    sock.close()

    intervals_ms = [(stamps[i] - stamps[i-1]) * 1000 for i in range(1, len(stamps))]
    ideal_ms     = 1000.0 / target_hz

    # Build a 5-bucket histogram
    deviations = [abs(x - ideal_ms) for x in intervals_ms]
    buckets    = [0.0, 0.1, 0.5, 1.0, 2.0, float("inf")]
    hist       = [0] * (len(buckets) - 1)
    for d in deviations:
        for i in range(len(buckets) - 1):
            if buckets[i] <= d < buckets[i+1]:
                hist[i] += 1
                break

    total = len(intervals_ms)
    return {
        "target_hz":    target_hz,
        "n":            total,
        "ideal_ms":     ideal_ms,
        "avg_ms":       statistics.mean(intervals_ms),
        "std_ms":       statistics.stdev(intervals_ms) if total > 1 else 0.0,
        "min_ms":       min(intervals_ms),
        "max_ms":       max(intervals_ms),
        "p95_ms":       sorted(intervals_ms)[int(total * 0.95)],
        "hist":         hist,
        "buckets":      buckets,
        "within_01ms_pct": hist[0] / total * 100,
        "within_1ms_pct":  sum(hist[:3]) / total * 100,
    }


def test_jitter() -> None:
    _hdr("4. Jitter Analysis at 30 Hz (5 seconds)")
    r = measure_jitter(target_hz=30, duration_s=5.0)
    _row("Intervals measured",   f"{r['n']:,}")
    _row("Ideal interval",       f"{r['ideal_ms']:.3f} ms")
    _row("Actual avg interval",  f"{r['avg_ms']:.3f} ms")
    _row("Jitter (std-dev)",     f"{r['std_ms']:.3f} ms")
    _row("Min / Max interval",   f"{r['min_ms']:.3f} ms  /  {r['max_ms']:.3f} ms")
    _row("p95 interval",         f"{r['p95_ms']:.3f} ms")
    _row("Within ±0.1 ms",       f"{r['within_01ms_pct']:.1f} %")
    _row("Within ±1.0 ms",       f"{r['within_1ms_pct']:.1f} %")
    print()
    print("  Deviation histogram:")
    labels = ["< 0.1 ms", "0.1–0.5 ms", "0.5–1.0 ms", "1–2 ms", "≥ 2 ms"]
    total  = r["n"]
    for label, count in zip(labels, r["hist"]):
        pct = count / total * 100
        bar = "█" * int(pct / 2)
        print(f"    {label:<12}  {pct:5.1f}%  {bar}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="ROV UDP speed test suite")
    parser.add_argument("--rtt",    action="store_true")
    parser.add_argument("--rate",   action="store_true")
    parser.add_argument("--burst",  action="store_true")
    parser.add_argument("--jitter", action="store_true")
    args = parser.parse_args()

    run_all = not any([args.rtt, args.rate, args.burst, args.jitter])

    print(f"\n  Packet size: {len(_SAMPLE_PKT)} bytes  |  Target: {LOOPBACK}")

    if run_all or args.rtt:    test_rtt()
    if run_all or args.rate:   test_send_rate()
    if run_all or args.burst:  test_burst()
    if run_all or args.jitter: test_jitter()

    print(f"\n{'═'*62}\n  Done.\n{'═'*62}\n")


if __name__ == "__main__":
    main()
