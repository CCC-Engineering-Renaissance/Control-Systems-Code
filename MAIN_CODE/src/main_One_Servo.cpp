#include <algorithm>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>
#include "PCA9685.h"
#include "Thruster.h"
#include "Thruster_Mixer.h"
#include "Claw.h"
#include "connection.h"
#include "PID.h"
#include "IMU.h"

namespace Config {
  constexpr bool kFrontLeftHorizontal  = true;
  constexpr bool kFrontRightHorizontal = true;
  constexpr bool kRearLeftHorizontal   = true;
  constexpr bool kRearRightHorizontal  = true;
  constexpr bool kLeftVertical         = true;
  constexpr bool kRightVertical        = false;
  constexpr bool kLeftVertical2        = false;
  constexpr bool kRightVertical2       = true;
  constexpr bool kClawRotate           = true;
  constexpr bool kClawOpen             = true;
  constexpr bool kPID                  = false;
  constexpr bool kIMU                  = false;
}

namespace {
  volatile std::sig_atomic_t keepRunning = 1;
  constexpr unsigned short kPort        = 5005;
  constexpr int   kStalePacketMs        = 500;
  constexpr float kMaxDt                = 0.1f;
  constexpr int   kArmDelayMs           = 3000;

  constexpr int   kChClawRotate = 8;
  constexpr int   kChClawOpen   = 9;
  constexpr int   kClawOffset   = 364;
  constexpr int   kClawRest     = 1500 - (0 * kClawOffset);
  constexpr int   kClawMinUs    = kClawOffset;
  constexpr int   kClawMaxUs    = 1500 + kClawOffset;
  constexpr float kClawSpeed    = 1.5f;
  constexpr float kMaxThrustCoeff = 0.8f;
  constexpr float kSlowModePercent = 0.2f;

  void signalHandler(int) { keepRunning = 0; }
}

static void stopThruster(bool enabled, Thruster& t, PiPCA9685::PCA9685& d) {
  if (enabled) t.stop(d);
}
static void setPowerThruster(bool enabled, Thruster& t, float power, PiPCA9685::PCA9685& d) {
  if (enabled) t.setPower(power, d);
}
static void centerClaw(bool enabled, Claw& c, PiPCA9685::PCA9685& d) {
  if (enabled) c.center(d);
}
static void setPosClaw(bool enabled, Claw& c, float pos, PiPCA9685::PCA9685& d) {
  if (enabled) c.setPosition(pos, d);
}

int main() {
  std::signal(SIGINT,  signalHandler);
  std::signal(SIGTERM, signalHandler);
  std::signal(SIGHUP,  signalHandler);
  std::signal(SIGQUIT, signalHandler);
  std::cout << "ROV starting...\n";

  std::cout << "── Active hardware ──────────────────\n";
  std::cout << "  FrontLeftH  : " << (Config::kFrontLeftHorizontal  ? "ON" : "OFF") << "\n";
  std::cout << "  FrontRightH : " << (Config::kFrontRightHorizontal ? "ON" : "OFF") << "\n";
  std::cout << "  RearLeftH   : " << (Config::kRearLeftHorizontal   ? "ON" : "OFF") << "\n";
  std::cout << "  RearRightH  : " << (Config::kRearRightHorizontal  ? "ON" : "OFF") << "\n";
  std::cout << "  LeftVert    : " << (Config::kLeftVertical         ? "ON" : "OFF") << "\n";
  std::cout << "  RightVert   : " << (Config::kRightVertical        ? "ON" : "OFF") << "\n";
  std::cout << "  LeftVert2   : " << (Config::kLeftVertical2        ? "ON" : "OFF") << "\n";
  std::cout << "  RightVert2  : " << (Config::kRightVertical2       ? "ON" : "OFF") << "\n";
  std::cout << "  Servo1 (ch8): " << (Config::kClawRotate           ? "ON" : "OFF") << "\n";
  std::cout << "  Servo2 (ch9): " << (Config::kClawOpen             ? "ON" : "OFF") << "\n";
  std::cout << "  PID         : " << (Config::kPID                  ? "ON" : "OFF") << "\n";
  std::cout << "  IMU         : " << (Config::kIMU                  ? "ON" : "OFF") << "\n";
  std::cout << "─────────────────────────────────────\n";

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

  // Let PCA9685 oscillator stabilize before doing anything
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // ── Thruster & Claw declarations ─────────────────────────────────────────
  Thruster frontLeftHorizontal(0);
  Thruster frontRightHorizontal(1);
  Thruster rearLeftHorizontal(6);
  Thruster rearRightHorizontal(7);

  Thruster leftVertical(2);
  Thruster rightVertical(3);
  Thruster leftVertical2(4);
  Thruster rightVertical2(5);

  leftVertical.setInverted(true);
  rightVertical2.setInverted(true);
  rearLeftHorizontal.setInverted(true);
  rearRightHorizontal.setInverted(true);

  Claw clawRotate(kChClawRotate, kClawRest, kClawOffset);
  clawRotate.setLimits(kClawMinUs, kClawMaxUs);

  Claw clawOpen(kChClawOpen, kClawRest, kClawOffset);
  clawOpen.setLimits(kClawMinUs, kClawMaxUs);

  // ── SafeStop guard ───────────────────────────────────────────────────────
  struct SafeStop {
    PiPCA9685::PCA9685& drv;
    Thruster &flh, &frh, &rlh, &rrh, &lv, &rv, &lv2, &rv2;
    Claw &cr, &cp;
    ~SafeStop() noexcept {
      auto safe = [](auto fn) noexcept { try { fn(); } catch (...) {} };
      safe([&]{ stopThruster(Config::kFrontLeftHorizontal,  flh,  drv); });
      safe([&]{ stopThruster(Config::kFrontRightHorizontal, frh,  drv); });
      safe([&]{ stopThruster(Config::kRearLeftHorizontal,   rlh,  drv); });
      safe([&]{ stopThruster(Config::kRearRightHorizontal,  rrh,  drv); });
      safe([&]{ stopThruster(Config::kLeftVertical,         lv,   drv); });
      safe([&]{ stopThruster(Config::kRightVertical,        rv,   drv); });
      safe([&]{ stopThruster(Config::kLeftVertical2,        lv2,  drv); });
      safe([&]{ stopThruster(Config::kRightVertical2,       rv2,  drv); });
      safe([&]{ centerClaw(Config::kClawRotate, cr, drv); });
      safe([&]{ centerClaw(Config::kClawOpen,   cp, drv); });
    }
  } guard{driver,
    frontLeftHorizontal, frontRightHorizontal, rearLeftHorizontal, rearRightHorizontal,
    leftVertical, rightVertical, leftVertical2, rightVertical2,
    clawRotate, clawOpen};
  /*

  // ── ESC Calibration sequence ─────────────────────────────────────────────
  std::cout << "CALIBRATION START\n";

  // Step 1: Send max throttle
  std::cout << "  Sending MAX throttle...\n";
  setPowerThruster(Config::kFrontLeftHorizontal,  frontLeftHorizontal,  1.0f, driver);
  setPowerThruster(Config::kFrontRightHorizontal, frontRightHorizontal, 1.0f, driver);
  setPowerThruster(Config::kRearLeftHorizontal,   rearLeftHorizontal,   1.0f, driver);
  setPowerThruster(Config::kRearRightHorizontal,  rearRightHorizontal,  1.0f, driver);
  setPowerThruster(Config::kLeftVertical,         leftVertical,         1.0f, driver);
  setPowerThruster(Config::kRightVertical,        rightVertical,        1.0f, driver);
  setPowerThruster(Config::kLeftVertical2,        leftVertical2,        1.0f, driver);
  setPowerThruster(Config::kRightVertical2,       rightVertical2,       1.0f, driver);
  std::cout << "  MAX PWM value: " << frontLeftHorizontal.getPWM() << "us\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  // Step 2: Send min throttle
  std::cout << "  Sending MIN throttle...\n";
  setPowerThruster(Config::kFrontLeftHorizontal,  frontLeftHorizontal,  -1.0f, driver);
  setPowerThruster(Config::kFrontRightHorizontal, frontRightHorizontal, -1.0f, driver);
  setPowerThruster(Config::kRearLeftHorizontal,   rearLeftHorizontal,   -1.0f, driver);
  setPowerThruster(Config::kRearRightHorizontal,  rearRightHorizontal,  -1.0f, driver);
  setPowerThruster(Config::kLeftVertical,         leftVertical,         -1.0f, driver);
  setPowerThruster(Config::kRightVertical,        rightVertical,        -1.0f, driver);
  setPowerThruster(Config::kLeftVertical2,        leftVertical2,        -1.0f, driver);
  setPowerThruster(Config::kRightVertical2,       rightVertical2,       -1.0f, driver);
  std::cout << "  MIN PWM value: " << frontLeftHorizontal.getPWM() << "us\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  // Step 3: Send neutral
  std::cout << "  Sending NEUTRAL...\n";
  stopThruster(Config::kFrontLeftHorizontal,  frontLeftHorizontal,  driver);
  stopThruster(Config::kFrontRightHorizontal, frontRightHorizontal, driver);
  stopThruster(Config::kRearLeftHorizontal,   rearLeftHorizontal,   driver);
  stopThruster(Config::kRearRightHorizontal,  rearRightHorizontal,  driver);
  stopThruster(Config::kLeftVertical,         leftVertical,         driver);
  stopThruster(Config::kRightVertical,        rightVertical,        driver);
  stopThruster(Config::kLeftVertical2,        leftVertical2,        driver);
  stopThruster(Config::kRightVertical2,       rightVertical2,       driver);
  centerClaw(Config::kClawRotate, clawRotate, driver);
  centerClaw(Config::kClawOpen,   clawOpen,   driver);
  std::cout << "  NEUTRAL PWM value: " << frontLeftHorizontal.getPWM() << "us\n";
  std::cout << "CALIBRATION COMPLETE - waiting for arm...\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(kArmDelayMs));
 */

  std::cout << "ROV is ON\n";

  std::cout << "── Network interfaces ───────────────\n";
  std::system("ip -brief addr show | grep -E 'eth0|wlan0|end0' || echo '  No eth/wlan interfaces found'");
  std::cout << "── UDP socket bound on port " << kPort << " ──\n";
  std::cout << "─────────────────────────────────────\n";

  Thruster_Mixer mixer;
  PID yawPID  (0.02f, 0.0f, 0.01f, -1.0f, 1.0f);
  PID pitchPID(0.02f, 0.0f, 0.01f, -1.0f, 1.0f);
  PID rollPID (0.02f, 0.0f, 0.01f, -1.0f, 1.0f);

  std::unique_ptr<IMU> imu;
  if (Config::kIMU) {
    imu = std::make_unique<IMU>();
    std::cout << "IMU initialized at 0x68\n";
  }

  float clawRotatePos = -1.0f;
  float clawOpenPos   = 0.0f;

  auto lastTime = std::chrono::steady_clock::now();
  bool wasWaiting = true;

  try {
  while (keepRunning) {
    setPosClaw(Config::kClawRotate, clawRotate, clawRotatePos, driver);
    setPosClaw(Config::kClawOpen,  clawOpen,  clawOpenPos,  driver);

    if (!is_Fresh(kStalePacketMs)) {
      stopThruster(Config::kFrontLeftHorizontal,  frontLeftHorizontal,  driver);
      stopThruster(Config::kFrontRightHorizontal, frontRightHorizontal, driver);
      stopThruster(Config::kRearLeftHorizontal,   rearLeftHorizontal,   driver);
      stopThruster(Config::kRearRightHorizontal,  rearRightHorizontal,  driver);
      stopThruster(Config::kLeftVertical,         leftVertical,         driver);
      stopThruster(Config::kRightVertical,        rightVertical,        driver);
      stopThruster(Config::kLeftVertical2,        leftVertical2,        driver);
      stopThruster(Config::kRightVertical2,       rightVertical2,       driver);
      yawPID.reset();
      pitchPID.reset();
      rollPID.reset();
      lastTime = std::chrono::steady_clock::now();
      if (!wasWaiting) {
        std::cout << "Waiting for controller packets...\n";
        wasWaiting = true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - lastTime).count();
    lastTime  = now;
    dt        = std::min(dt, kMaxDt);

    if (wasWaiting) {
      std::cout << "Controller packet received.\n";
      wasWaiting = false;
    }

    const POVState input = get_State();

    clawRotatePos = std::clamp(clawRotatePos + input.clawRotate * kClawSpeed * dt, -1.0f, 1.0f);
    clawOpenPos   = std::clamp(clawOpenPos   + input.clawOpen   * kClawSpeed * dt, -1.0f, 1.0f);

    float measuredPitch = 0.0f;
    float measuredYaw   = 0.0f;
    float measuredRoll  = 0.0f;

    if (Config::kIMU && imu) {
      imu->update();
      measuredPitch = imu->getPitch();
      measuredYaw   = imu->getYaw();
      measuredRoll  = imu->getRoll();
    }

    float yawPIDOutput   = 0.0f;
    float pitchPIDOutput = 0.0f;
    float rollPIDOutput  = 0.0f;

    if (Config::kPID && input.als) {
      pitchPIDOutput = pitchPID.update(input.pitchAngle, measuredPitch, dt);
      yawPIDOutput   = yawPID  .update(input.yawAngle,   measuredYaw,   dt);
      rollPIDOutput  = rollPID .update(0.0f,             measuredRoll,  dt);
    } else {
      yawPID.reset();
      pitchPID.reset();
      rollPID.reset();
    }

    POVState mixInput = input;
    mixInput.forward = -mixInput.forward;
    mixInput.yaw = -mixInput.yaw;
    if (Config::kPID && input.als) {
      mixInput.yaw   = 0.0f;
      mixInput.pitch = 0.0f;
    }

    const Thruster_Outputs output =
        mixer.mix(mixInput, yawPIDOutput, pitchPIDOutput, rollPIDOutput);

    setPowerThruster(Config::kFrontLeftHorizontal,  frontLeftHorizontal,  output.frontLeftHorizontal * kMaxThrustCoeff,  driver);
    setPowerThruster(Config::kFrontRightHorizontal, frontRightHorizontal, output.frontRightHorizontal * kMaxThrustCoeff, driver);
    setPowerThruster(Config::kRearLeftHorizontal,   rearLeftHorizontal,   output.rearLeftHorizontal * kMaxThrustCoeff,   driver);
    setPowerThruster(Config::kRearRightHorizontal,  rearRightHorizontal,  output.rearRightHorizontal * kMaxThrustCoeff,  driver);
    setPowerThruster(Config::kLeftVertical,         leftVertical,         output.leftVertical * kMaxThrustCoeff,         driver);
    setPowerThruster(Config::kRightVertical,        rightVertical,        output.rightVertical * kMaxThrustCoeff,        driver);
    setPowerThruster(Config::kLeftVertical2,        leftVertical2,        output.leftVertical2 * kMaxThrustCoeff,        driver);
    setPowerThruster(Config::kRightVertical2,       rightVertical2,       output.rightVertical2 * kMaxThrustCoeff,       driver);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  } catch (const std::exception& e) {
    std::cerr << "\nControl loop exception: " << e.what() << "\n";
  }

  stopServer();
  net.join();
  std::cout << "\nExiting safely.\n";
  return 0;
}
