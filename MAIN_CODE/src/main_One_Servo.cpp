#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include "PCA9685.h"
#include "Claw.h"
#include "connection.h"

namespace {
  volatile std::sig_atomic_t keepRunning = 1;
  constexpr unsigned short kPort      = 5005;
  constexpr int   kStalePacketMs      = 250;
  constexpr int   kArmDelayMs         = 500;

  constexpr int kChClaw   = 8;
  constexpr int kClawRest = 1500;
  constexpr int kClawOffset = 556;
  constexpr int kClawMinUs  = 944;
  constexpr int kClawMaxUs  = 2056;

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

  while (keepRunning) {
    if (!is_Fresh(kStalePacketMs)) {
      servo.center(driver);
      std::cout << "Waiting for packets...\r";
      std::cout.flush();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    const POVState input = get_State();
    servo.setPosition(input.clawRotate, driver);

    std::cout << "Servo position: " << input.clawRotate << "    \r";
    std::cout.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  servo.center(driver);
  net.detach();
  std::cout << "\nExiting safely.\n";
  return 0;
}
