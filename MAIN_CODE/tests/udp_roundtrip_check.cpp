// udp_roundtrip_check.cpp — end-to-end check that real packets produced by
// thruster.py's _build_packet() are received and parsed correctly by the
// actual ROV-side UDP server (connection.cpp).
//
// Run together with tests/send_test_packet.py:
//   ./out/udp_roundtrip_check &  python3 tests/send_test_packet.py
//
// The sender transmits a known test vector; this program waits for it and
// verifies every POVState field.

#include "connection.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

namespace {

constexpr unsigned short kTestPort = 5007;

bool approx(float a, float b) { return std::fabs(a - b) < 1e-3f; }

int check(const char* name, float got, float want) {
  if (!approx(got, want)) {
    std::cerr << "FAILED: " << name << " got " << got
              << " want " << want << "\n";
    return 1;
  }
  std::cout << "  ok: " << name << " = " << got << "\n";
  return 0;
}

}  // namespace

int main() {
  std::thread net([] { server(kTestPort); });

  // Wait up to 5 s for the Python sender's packet to arrive.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!is_Fresh(250) && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  int failures = 0;
  if (!is_Fresh(2000)) {
    std::cerr << "FAILED: no packet received within 5 s\n";
    failures = 1;
  } else {
    const POVState s = get_State();
    // Must match the test vector in tests/send_test_packet.py
    failures += check("forward       (left stick up)   ", s.forward,        0.5f);
    failures += check("strafe        (left stick left)  ", s.strafe,        -0.25f);
    failures += check("vertical      (right trigger)    ", s.vertical,       0.6f);
    failures += check("yaw           (right stick right)", s.yaw,            0.3f);
    failures += check("pitch         (right stick back) ", s.pitch,          0.2f);
    failures += check("roll          (right bumper)     ", s.roll,           0.75f);
    failures += check("clawRotate                       ", s.clawRotate,     1.0f);
    failures += check("clawOpen                         ", s.clawOpen,      -1.0f);
    failures += check("clawBrushless                    ", s.clawBrushless,  0.9f);
    failures += check("pitchAngle                       ", s.pitchAngle,     0.1f);
    failures += check("yawAngle                         ", s.yawAngle,      -0.2f);
    if (!s.als) {
      std::cerr << "FAILED: als should be true\n";
      ++failures;
    } else {
      std::cout << "  ok: als = true\n";
    }
  }

  stopServer();
  net.join();

  if (failures) {
    std::cerr << failures << " field(s) failed\n";
    return 1;
  }
  std::cout << "udp_roundtrip_check passed\n";
  return 0;
}
