#include "PCA9685.h"
#include "Thruster.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

bool approx(double a, double b, double eps = 1e-6) {
  return std::fabs(a - b) <= eps;
}

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void test_default_mapping() {
  PiPCA9685::PCA9685 driver;
  Thruster thruster(3);

  thruster.setPower(1.0, driver);
  require(thruster.getPWM() == 1900, "full forward should map to 1900 us");
  require(approx(thruster.getPower(), 1.0), "cached power should be +1");
  require(driver.last_channel == 3, "driver should receive the thruster channel");
  require(approx(driver.last_ms, 1.9), "1900 us should be written as 1.9 ms");

  thruster.setPower(-1.0, driver);
  require(thruster.getPWM() == 1100, "full reverse should map to 1100 us");
  require(approx(thruster.getPower(), -1.0), "cached power should be -1");
  require(approx(driver.last_ms, 1.1), "1100 us should be written as 1.1 ms");
}

void test_clamping_and_stop() {
  PiPCA9685::PCA9685 driver;
  Thruster thruster(4);

  thruster.setPower(2.0, driver);
  require(thruster.getPWM() == 1900, "power should clamp above +1");

  thruster.setPWM(2500, driver);
  require(thruster.getPWM() == 1900, "PWM should clamp to max safety limit");

  thruster.setPWM(500, driver);
  require(thruster.getPWM() == 1100, "PWM should clamp to min safety limit");

  thruster.stop(driver);
  require(thruster.getPWM() == 1500, "stop should return to neutral");
  require(approx(thruster.getPower(), 0.0), "stop should reset power to zero");
}

void test_configuration_updates() {
  PiPCA9685::PCA9685 driver;
  Thruster thruster(5, 1520, 300);

  thruster.setPower(1.0, driver);
  require(thruster.getPWM() == 1820, "custom rest/offset should affect mapping");

  thruster.setLimits(1700, 1300);
  thruster.setPWM(2000, driver);
  require(thruster.getPWM() == 1700, "reversed limits should be normalized");
  require(approx(thruster.getPower(), 0.6), "power cache should match clamped PWM");

  thruster.setOffset(0);
  thruster.setPWM(1520, driver);
  require(approx(thruster.getPower(), 0.0), "zero offset should avoid divide-by-zero");
}

void test_invalid_pin_rejected() {
  bool threw = false;
  try {
    Thruster thruster(16);
    (void)thruster;
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  require(threw, "invalid pins should throw");
}

}  // namespace

int main() {
  test_default_mapping();
  test_clamping_and_stop();
  test_configuration_updates();
  test_invalid_pin_rejected();
  std::cout << "thruster_logic_test passed\n";
  return 0;
}
