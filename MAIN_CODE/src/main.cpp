#include <algorithm>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include "PCA9685.h"
#include "Thruster.h"
#include "Thruster_Mixer.h"
#include "Claw.h"
#include "connection.h"
#include "PID.h"

namespace {
  volatile std::sig_atomic_t keepRunning = 1;
  constexpr unsigned short kPort        = 5005;
  constexpr int   kStalePacketMs        = 250;
  constexpr float kMaxDt                = 0.1f;
  constexpr int   kArmDelayMs           = 500;

  constexpr int   kChClawRotate = 8;
  constexpr int   kChClawOpen   = 9;
  constexpr int   kClawRest     = 1500;
  constexpr int   kClawOffset   = 556;
  constexpr int   kClawMinUs    = 944;
  constexpr int   kClawMaxUs    = 2056;
  constexpr float kClawSpeed    = 0.5f; // full travel in 2 s at full button press

  void signalHandler(int) {
    keepRunning = 0;
  }
} // namespace

int main() {
  std::signal(SIGINT, signalHandler);
  std::cout << "ROV starting...\n";
  std::cout << "Listening for UDP on port " << kPort << "\n";

  std::thread net([] {
    try {
      server(kPort);
    } catch (const std::exception& e) {
      std::cerr << "Network thread exception: " << e.what() << "\n";
    } catch (...) {
      std::cerr << "Network thread unknown exception\n";
    }
  });

  PiPCA9685::PCA9685 driver("/dev/i2c-1", 0x40);
  driver.set_pwm_freq(50.0);
  std::cout << "PCA9685 initialized on /dev/i2c-1 at address 0x40\n";

  Thruster frontLeftHorizontal(0);
  Thruster frontRightHorizontal(1);
  Thruster rearLeftHorizontal(2);
  Thruster rearRightHorizontal(3);
  Thruster leftVertical(4);
  Thruster rightVertical(5);
  Thruster leftVertical2(6);
  Thruster rightVertical2(7);

  Claw clawRotate(kChClawRotate, kClawRest, kClawOffset);
  Claw clawOpen  (kChClawOpen,   kClawRest, kClawOffset);
  clawRotate.setLimits(kClawMinUs, kClawMaxUs);
  clawOpen.setLimits  (kClawMinUs, kClawMaxUs);

  // Send neutral immediately so ESCs don't see garbage PWM on boot
  frontLeftHorizontal.stop(driver);
  frontRightHorizontal.stop(driver);
  rearLeftHorizontal.stop(driver);
  rearRightHorizontal.stop(driver);
  leftVertical.stop(driver);
  rightVertical.stop(driver);
  leftVertical2.stop(driver);
  rightVertical2.stop(driver);
  clawRotate.center(driver);
  clawOpen.center  (driver);
  std::cout << "All thrusters set to neutral, waiting for ESCs to arm...\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(kArmDelayMs));
  std::cout << "ROV is ON\n";

  Thruster_Mixer mixer;
  PID yawPID  (0.02f, 0.0f, 0.01f, -1.0f, 1.0f);
  PID pitchPID(0.02f, 0.0f, 0.01f, -1.0f, 1.0f);
  PID rollPID (0.02f, 0.0f, 0.01f, -1.0f, 1.0f);

  // Current servo positions in [-1, 1]; updated incrementally each loop
  float clawRotatePos = 0.0f;
  float clawOpenPos   = 0.0f;

  auto lastTime = std::chrono::steady_clock::now();

  while (keepRunning) {
    if (!is_Fresh(kStalePacketMs)) {
      frontLeftHorizontal.stop(driver);
      frontRightHorizontal.stop(driver);
      rearLeftHorizontal.stop(driver);
      rearRightHorizontal.stop(driver);
      leftVertical.stop(driver);
      rightVertical.stop(driver);
      leftVertical2.stop(driver);
      rightVertical2.stop(driver);
      clawRotate.center(driver);
      clawOpen.center  (driver);
      clawRotatePos = 0.0f;
      clawOpenPos   = 0.0f;
      yawPID.reset();
      pitchPID.reset();
      rollPID.reset();
      lastTime = std::chrono::steady_clock::now();
      std::cout << "Waiting for controller packets...\r";
      std::cout.flush();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - lastTime).count();
    lastTime  = now;
    dt        = std::min(dt, kMaxDt);

    const POVState input = get_State();

    // NOTE: measured values are placeholders — replace with real IMU readings
    float measuredPitch = 0.0f;
    float measuredYaw   = 0.0f;
    float measuredRoll  = 0.0f;

    float yawPIDOutput   = 0.0f;
    float pitchPIDOutput = 0.0f;
    float rollPIDOutput  = 0.0f;

    if (input.als) {
      pitchPIDOutput = pitchPID.update(input.pitchAngle, measuredPitch, dt);
      yawPIDOutput   = yawPID  .update(input.yawAngle,   measuredYaw,   dt);
      rollPIDOutput  = rollPID .update(0.0f,             measuredRoll,  dt);
    } else {
      yawPID.reset();
      pitchPID.reset();
      rollPID.reset();
    }

    const Thruster_Outputs output =
        mixer.mix(input, yawPIDOutput, pitchPIDOutput, rollPIDOutput);

    frontLeftHorizontal.setPower(output.frontLeftHorizontal,   driver);
    frontRightHorizontal.setPower(output.frontRightHorizontal, driver);
    rearLeftHorizontal.setPower(output.rearLeftHorizontal,     driver);
    rearRightHorizontal.setPower(output.rearRightHorizontal,   driver);
    leftVertical.setPower(output.leftVertical,                 driver);
    rightVertical.setPower(output.rightVertical,               driver);
    leftVertical2.setPower(output.leftVertical2,               driver);
    rightVertical2.setPower(output.rightVertical2,             driver);

    // Button held => move; button released (input == 0) => hold position
    clawRotatePos = std::clamp(clawRotatePos + input.clawRotate * kClawSpeed * dt, -1.0f, 1.0f);
    clawOpenPos   = std::clamp(clawOpenPos   + input.clawOpen   * kClawSpeed * dt, -1.0f, 1.0f);
    clawRotate.setPosition(clawRotatePos, driver);
    clawOpen.setPosition  (clawOpenPos,   driver);

    std::cout << "ROV running...                    \r";
    std::cout.flush();
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
  clawRotate.center(driver);
  clawOpen.center  (driver);
  net.detach();
  std::cout << "\nExiting safely.\n";
  return 0;
}
