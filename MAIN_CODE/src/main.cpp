#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include "PCA9685.h"
#include "Thruster.h"
#include "Thruster_Mixer.h"
#include "connection.h"

namespace {

volatile std::sig_atomic_t keepRunning = 1;
constexpr unsigned short kPort = 12345;

void signalHandler(int) {
  keepRunning = 0;
}

}  // namespace

int main() {
  std::signal(SIGINT, signalHandler);

  std::thread net([] { server(kPort); });

  PiPCA9685::PCA9685 driver("/dev/i2c-1", 0x40);
  driver.set_pwm_freq(50.0);

  Thruster frontLeftHorizontal(0);
  Thruster frontRightHorizontal(1);
  Thruster rearLeftHorizontal(2);
  Thruster rearRightHorizontal(3);
  Thruster leftVertical(4);
  Thruster rightVertical(5);
  Thruster leftVertical2(6);
  Thruster rightVertical2(7);

  Thruster_Mixer mixer;

  while (keepRunning) {
    if (!is_Fresh(250)) {
      frontLeftHorizontal.stop(driver);
      frontRightHorizontal.stop(driver);
      rearLeftHorizontal.stop(driver);
      rearRightHorizontal.stop(driver);
      leftVertical.stop(driver);
      rightVertical.stop(driver);
      leftVertical2.stop(driver);
      rightVertical2.stop(driver);

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    const POVState input = get_State();
    const Thruster_Outputs output = mixer.mix(input);

    frontLeftHorizontal.setPower(output.frontLeftHorizontal, driver);
    frontRightHorizontal.setPower(output.frontRightHorizontal, driver);
    rearLeftHorizontal.setPower(output.rearLeftHorizontal, driver);
    rearRightHorizontal.setPower(output.rearRightHorizontal, driver);
    leftVertical.setPower(output.leftVertical, driver);
    rightVertical.setPower(output.rightVertical, driver);
    leftVertical2.setPower(output.leftVertical2, driver);
    rightVertical2.setPower(output.rightVertical2, driver);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  frontLeftHorizontal.stop(driver);
  frontRightHorizontal.stop(driver);
  rearLeftHorizontal.stop(driver);
  rearRightHorizontal.stop(driver);
  leftVertical.stop(driver);
  rightVertical.stop(driver);
  leftVertical2.stop(driver);
  rightVertical2.stop(driver);

  net.detach();
  std::cout << "Exiting safely.\n";
  return 0;
}
