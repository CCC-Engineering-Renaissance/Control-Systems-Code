#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include "PCA9685.h"
#include "Claw.h"
#include "connection.h"

namespace {
  volatile std::sig_atomic_t keepRunning = 1;
  constexpr unsigned short kPort       = 5005;
  constexpr int   kStalePacketMs       = 250;
  constexpr int   kArmDelayMs          = 500;
  constexpr int kChClaw    = 8;
  constexpr int kClawRest  = 1500;
  constexpr int kClawOffset = 100;
  constexpr int kClawMinUs  = 500;
  constexpr int kClawMaxUs  = 2500;

  // Claw open/closed positions — tune these to your servo's range
  constexpr int kClawOpen   = kClawRest + kClawOffset;  // or a specific us value
  constexpr int kClawClosed = kClawRest - kClawOffset;

  void signalHandler(int) { keepRunning = 0; }
}

int main() {
  std::signal(SIGINT, signalHandler);
  std::cout << "Starting single servo test...\n";

  std::thread net([] {
    try { server(kPort); }
    catch (const std::exception& e) {
      std::cerr << "Network thread exception: " << e.what() << "\n";
    }
  });

  PiPCA9685::PCA9685 driver("/dev/i2c-1", 0x40);
  driver.set_pwm_freq(50.0);

  Claw servo(kChClaw, kClawRest, kClawOffset);
  servo.setLimits(kClawMinUs, kClawMaxUs);
  servo.center(driver);

  std::cout << "Servo centered, waiting for ESC arm...\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(kArmDelayMs));
  std::cout << "Ready\n";

  bool clawOpen    = false;  // tracks current toggle state
  bool prevButtonA = false;  // tracks last A button state for edge detection

  while (keepRunning) {
    if (!is_Fresh(kStalePacketMs)) {
      servo.center(driver);
      std::cout << "Waiting for packets...\r";
      std::cout.flush();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      prevButtonA = false;  // reset edge detection when stale
      continue;
    }

    const POVState input = get_State();

    // --- A button toggle (rising-edge detection) ---
    // Change `input.buttonA` to match your actual POVState field name
    const bool currButtonA = input.clawOpen;
    if (currButtonA && !prevButtonA) {
      clawOpen = !clawOpen;
      const int targetUs = clawOpen ? kClawOpen : kClawClosed;
      servo.setPosition(targetUs, driver);
      std::cout << "Claw " << (clawOpen ? "OPEN" : "CLOSED") << "        \n";
    }
    prevButtonA = currButtonA;

    std::cout << "A=" << currButtonA << " clawOpen=" << clawOpen << "    \r";
    std::cout.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  servo.center(driver);
  net.detach();
  std::cout << "\nExiting safely.\n";
  return 0;
}
