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
  constexpr bool kRightVertical        = true;
  constexpr bool kLeftVertical2        = true;
  constexpr bool kRightVertical2       = true;
  constexpr bool kClawRotate = true;
  constexpr bool kClawOpen  = true;
  constexpr bool kPID        = false;
  constexpr bool kIMU        = false;
  constexpr bool kClawTest   = false;  // run spin-claw sweep on startup
}

namespace {
  volatile std::sig_atomic_t keepRunning = 1;
  constexpr unsigned short kPort        = 5005;
  constexpr int   kStalePacketMs        = 500;
  constexpr float kMaxDt                = 0.1f;
  constexpr int   kArmDelayMs           = 3000;

  constexpr int kChClawRotate = 8;
  constexpr int kChClawOpen  = 9;
  constexpr int kClawRest     = 1500;
  constexpr int kClawOffset   = 278;  // 50° per side (50/90 × 500 ≈ 278 µs)
  constexpr int kClawMinUs    = 1222; // 1500 - 278
  constexpr int kClawMaxUs    = 1778; // 1500 + 278

  constexpr float kClawStep = 0.05f;

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
  std::cout << "Listening for UDP on port " << kPort << "\n";

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
  std::cout << "  Servo2 (ch9): " << (Config::kClawOpen            ? "ON" : "OFF") << "\n";
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

  Thruster frontLeftHorizontal(0);
  Thruster frontRightHorizontal(1);
  Thruster leftVertical(2);
  Thruster rightVertical(3);
  Thruster leftVertical2(4);
  Thruster rightVertical2(5);
  Thruster rearLeftHorizontal(6);
  Thruster rearRightHorizontal(7);

  Claw clawRotate(kChClawRotate, kClawRest, kClawOffset);
  clawRotate.setLimits(kClawMinUs, kClawMaxUs);

  leftVertical.setInverted(true);
  rightVertical2.setInverted(true);
  frontRightHorizontal.setInverted(true);
  rearRightHorizontal.setInverted(true);

  Claw clawOpen(kChClawOpen, kClawRest, kClawOffset);
  clawOpen.setLimits(kClawMinUs, kClawMaxUs);

  // Commands all channels to neutral on any exit path (exception, signal, or normal return).
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
      safe([&]{ centerClaw(Config::kClawOpen,  cp, drv); });
    }
  } guard{driver,
    frontLeftHorizontal, frontRightHorizontal, rearLeftHorizontal, rearRightHorizontal,
    leftVertical, rightVertical, leftVertical2, rightVertical2,
    clawRotate, clawOpen};

  stopThruster(Config::kFrontLeftHorizontal,  frontLeftHorizontal,  driver);
  stopThruster(Config::kFrontRightHorizontal, frontRightHorizontal, driver);
  stopThruster(Config::kRearLeftHorizontal,   rearLeftHorizontal,   driver);
  stopThruster(Config::kRearRightHorizontal,  rearRightHorizontal,  driver);
  stopThruster(Config::kLeftVertical,         leftVertical,         driver);
  stopThruster(Config::kRightVertical,        rightVertical,        driver);
  stopThruster(Config::kLeftVertical2,        leftVertical2,        driver);
  stopThruster(Config::kRightVertical2,       rightVertical2,       driver);
  setPosClaw(Config::kClawRotate, clawRotate, -1.0f, driver);
  setPosClaw(Config::kClawOpen,  clawOpen,  -1.0f, driver);

  std::cout << "Active channels set to neutral, waiting for ESCs to arm...\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(kArmDelayMs));
  std::cout << "ROV is ON\n";

  // ── Network interface debug ───────────────────────────────────────────
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

  auto lastTime = std::chrono::steady_clock::now();

  float clawRotatePos = -1.0f;
  float clawOpenPos  = -1.0f;

  try {
  while (keepRunning) {
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
      std::cout << "Waiting for controller packets...\r";
      std::cout.flush();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - lastTime).count();
    lastTime  = now;
    dt        = std::min(dt, kMaxDt);

    const POVState input = get_State();

    // ── Claw rotate (servo, holds position) ───────────────────────────
    if (input.clawRotate > 0.0f) {
      clawRotatePos += kClawStep;
      if (clawRotatePos >= 1.0f) clawRotatePos = 1.0f;
      setPosClaw(Config::kClawRotate, clawRotate, clawRotatePos, driver);
    } else if (input.clawRotate < 0.0f) {
      clawRotatePos -= kClawStep;
      if (clawRotatePos <= -1.0f) clawRotatePos = -1.0f;
      setPosClaw(Config::kClawRotate, clawRotate, clawRotatePos, driver);
    }

    // LB → open clawOpen | LT → close clawOpen
    if (input.clawOpen > 0.0f) {
      clawOpenPos += kClawStep;
      if (clawOpenPos >= 1.0f) clawOpenPos = 1.0f;
      setPosClaw(Config::kClawOpen, clawOpen, clawOpenPos, driver);
    } else if (input.clawOpen < 0.0f) {
      clawOpenPos -= kClawStep;
      if (clawOpenPos <= -1.0f) clawOpenPos = -1.0f;
      setPosClaw(Config::kClawOpen, clawOpen, clawOpenPos, driver);
    }

    // ── Thrusters ──────────────────────────────────────────────────────
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

    // When ALS is active, PID handles yaw/pitch — don't also add raw stick
    POVState mixInput = input;
    if (Config::kPID && input.als) {
      mixInput.yaw   = 0.0f;
      mixInput.pitch = 0.0f;
    }

    const Thruster_Outputs output =
        mixer.mix(mixInput, yawPIDOutput, pitchPIDOutput, rollPIDOutput);

    std::cout << "V=" << input.vertical
              << " LV=" << output.leftVertical
              << " RV=" << output.rightVertical
              << " LV2=" << output.leftVertical2
              << " RV2=" << output.rightVertical2
              << "    \n";
    std::cout.flush();

    setPowerThruster(Config::kFrontLeftHorizontal,  frontLeftHorizontal,  output.frontLeftHorizontal,  driver);
    setPowerThruster(Config::kFrontRightHorizontal, frontRightHorizontal, output.frontRightHorizontal, driver);
    setPowerThruster(Config::kRearLeftHorizontal,   rearLeftHorizontal,   output.rearLeftHorizontal,   driver);
    setPowerThruster(Config::kRearRightHorizontal,  rearRightHorizontal,  output.rearRightHorizontal,  driver);
    setPowerThruster(Config::kLeftVertical,         leftVertical,         output.leftVertical,         driver);
    setPowerThruster(Config::kRightVertical,        rightVertical,        output.rightVertical,        driver);
    setPowerThruster(Config::kLeftVertical2,        leftVertical2,        output.leftVertical2,        driver);
    setPowerThruster(Config::kRightVertical2,       rightVertical2,       output.rightVertical2,       driver);

    std::cout << "ROV running | Servo1=" << clawRotatePos
              << " Servo2=" << clawOpenPos
              << "    \r";
    std::cout.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  } catch (const std::exception& e) {
    std::cerr << "\nControl loop exception: " << e.what() << "\n";
  }
  // SafeStop guard destructs here, commanding all channels to neutral.

  stopServer();
  net.join();
  std::cout << "\nExiting safely.\n";
  return 0;
}
