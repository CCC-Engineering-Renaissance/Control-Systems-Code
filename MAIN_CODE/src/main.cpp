#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include "PCA9685.h"
#include "Thruster.h"
#include "Thruster_Mixer.h"
#include "connection.h"
#include "PID.h"

namespace {

volatile std::sig_atomic_t keepRunning = 1;
constexpr unsigned short kPort = 5005;

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

  // Tune these values later on the real ROV
  PID yawPID(0.02f, 0.0f, 0.01f, -1.0f, 1.0f);
  PID pitchPID(0.02f, 0.0f, 0.01f, -1.0f, 1.0f);
  PID rollPID(0.02f, 0.0f, 0.01f, -1.0f, 1.0f);

  auto lastTime = std::chrono::steady_clock::now();

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

      yawPID.reset();
      pitchPID.reset();
      rollPID.reset();

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - lastTime).count();
    lastTime = now;

    const POVState input = get_State();

    // Replace these with real IMU readings
    float measuredPitch = 0.0f;
    float measuredYaw   = 0.0f;
    float measuredRoll  = 0.0f;

    float yawPIDOutput = 0.0f;
    float pitchPIDOutput = 0.0f;
    float rollPIDOutput = 0.0f;

    if (input.als) {
      // Uses target angles coming from thruster.py / connection.cpp
      pitchPIDOutput = pitchPID.update(input.pitchAngle, measuredPitch, dt);
      yawPIDOutput   = yawPID.update(input.yawAngle, measuredYaw, dt);

      // If you do not have a roll angle target yet, keep roll target at 0
      rollPIDOutput  = rollPID.update(0.0f, measuredRoll, dt);
    } else {
      yawPID.reset();
      pitchPID.reset();
      rollPID.reset();
    }

    const Thruster_Outputs output =
        mixer.mix(input, yawPIDOutput, pitchPIDOutput, rollPIDOutput);

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
