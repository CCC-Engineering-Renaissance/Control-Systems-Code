#include "../include/Claw.h"
#include "../include/PCA9685.h"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace {

int parseInt(const char* value, int fallback) {
  if (value == nullptr) {
    return fallback;
  }
  try {
    return std::stoi(value);
  } catch (...) {
    return fallback;
  }
}

void holdPosition(Claw& servo, PiPCA9685::PCA9685& driver, int pwm_us, int hold_ms) {
  servo.setPWM(pwm_us, driver);
  std::cout << "Commanded " << pwm_us << " us\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(hold_ms));
}

}  // namespace

int main(int argc, char** argv) {
  const int channel = (argc > 1) ? parseInt(argv[1], 9) : 9;
  const int rest_us = (argc > 2) ? parseInt(argv[2], 1500) : 1500;
  const int low_us = (argc > 3) ? parseInt(argv[3], 1100) : 1100;
  const int high_us = (argc > 4) ? parseInt(argv[4], 1900) : 1900;
  const int hold_ms = (argc > 5) ? parseInt(argv[5], 1500) : 1500;

  std::cout
      << "Testing PCA9685 channel " << channel
      << " at 50 Hz with pulses " << low_us << " / " << rest_us << " / " << high_us
      << " us\n";

  PiPCA9685::PCA9685 driver("/dev/i2c-1", 0x40);
  driver.set_pwm_freq(50.0);

  Claw servo(channel, rest_us, high_us - rest_us);
  servo.setLimits(low_us, high_us);

  // Step through three obvious positions so channel mapping and power issues
  // are easy to spot without the controller/network in the loop.
  holdPosition(servo, driver, rest_us, hold_ms);
  holdPosition(servo, driver, low_us, hold_ms);
  holdPosition(servo, driver, high_us, hold_ms);
  holdPosition(servo, driver, rest_us, hold_ms);

  return 0;
}
