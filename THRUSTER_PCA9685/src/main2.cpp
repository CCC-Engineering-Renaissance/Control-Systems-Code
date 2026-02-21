// main.cpp (single thruster, single axis)
// Right joystick Y (assumed mapped to s.pitch) drives thruster 0 power [-1, +1].
//
// Uses your existing connection layer:
//   - server(PORT) running in background
//   - is_Fresh(ms) watchdog
//   - get_State() snapshot
//
// Wiring: set THRUSTER_PIN to the PCA9685 channel your ESC is on.

#include "Constants.h"
#include "I2CPeripheral.h"
#include "PCA9685.h"
#include "Thruster.h"
#include "connections.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

using namespace std;

static inline double clampd(double v, double lo, double hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}
static inline float clamp1(float x) {
  if (x >  1.0f) return  1.0f;
  if (x < -1.0f) return -1.0f;
  return x;
}
constexpr unsigned shor PORT = 5005
int main() {
  // Start UDP server in background (your connection.cpp must provide this).
  std::thread net([&] { server(PORT); });
  net.detach();

  // PCA9685 driver
  PiPCA9685::PCA9685 driver("/dev/i2c-1", 0x40);
  driver.set_pwm_freq(50);

  // --- Single thruster setup ---
  constexpr int THRUSTER_PIN = 0;   // <-- CHANGE to your channel
  Thruster thr(THRUSTER_PIN);

  auto stop_thr = [&]() {
    thr.stop(driver);
  };

  cout << "Single-thruster teleop listening on UDP port " << PORT << "...\n";
  cout << "Right joystick Y -> thruster power\n";

  while (true) {
    // Safety stop if packets stop arriving
    if (!is_Fresh(250)) {
      stop_thr();
      cout << "STALE: stop\r";
      cout.flush();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    // Snapshot controller state
    POVState s = get_State();

    // --- Pick ONE axis: right joystick Y ---
    // Assumption: connection layer maps right joystick Y to s.pitch.
    // If yours differs, change this line to: s.vertical or s.forward etc.
    float axis = clamp1(s.pitch);

    // Optional deadband to prevent small drift
    const float DEAD = 0.08f;
    if (std::fabs(axis) < DEAD) axis = 0.0f;

    // Send to thruster
    double cmd = clampd(axis, -1.0, 1.0);
    thr.setPower(cmd, driver);

    // Debug
    cout << "RJ_Y=" << axis << " -> cmd=" << cmd << "     \r";
    cout.flush();

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  return 0;
}

