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

// ─────────────────────────────────────────────
//  HARDWARE CONFIG — set false for anything
//  not physically connected
// ─────────────────────────────────────────────
namespace Config {
  // Thrusters
  constexpr bool kFrontLeftHorizontal  = true;
  constexpr bool kFrontRightHorizontal = true;
  constexpr bool kRearLeftHorizontal   = true;
  constexpr bool kRearRightHorizontal  = true;
  constexpr bool kLeftVertical         = true;
  constexpr bool kRightVertical        = true;
  constexpr bool kLeftVertical2        = true;
  constexpr bool kRightVertical2       = true;

  // Claw servos
  constexpr bool kClawRotate = true;
  constexpr bool kClawOpen   = true;
  constexpr bool kClawPitch  = true;

  constexpr bool kPID = true;
}

namespace {
  volatile std::sig_atomic_t keepRunning = 1;
  constexpr unsigned short kPort        = 5005;
  constexpr int   kStalePacketMs        = 250;
  constexpr float kMaxDt                = 0.1f;
  constexpr int   kArmDelayMs           = 500;

  constexpr int kChClawRotate = 8;
  constexpr int kChClawOpen   = 9;
  constexpr int kChClawPitch  = 10;
  constexpr int kClawRest     = 1500;
  constexpr int kClawOffset   = 556;
  constexpr int kClawMinUs    = 944;
  constexpr int kClawMaxUs    = 2056;

  constexpr float kClawStep = 0.05f;  // position increment per 50ms tick
                                       // 1.0 = full travel in ~1 second

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
  std::signal(SIGINT, signalHandler);
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
  std::cout << "  ClawRotate  : " << (Config::kClawRotate           ? "ON" : "OFF") << "\n";
  std::cout << "  ClawOpen    : " << (Config::kClawOpen             ? "ON" : "OFF") << "\n";
  std::cout << "  ClawPitch   : " << (Config::kClawPitch            ? "ON" : "OFF") << "\n";
  std::cout << "  PID         : " << (Config::kPID                  ? "ON" : "OFF") << "\n";
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
  Thruster rearLeftHorizontal(2);
  Thruster rearRightHorizontal(3);
  Thruster leftVertical(4);
  Thruster rightVertical(5);
  Thruster leftVertical2(6);
  Thruster rightVertical2(7);

  Claw clawRotate   (kChClawRotate, kClawRest, kClawOffset);
  Claw clawOpenServo(kChClawOpen,   kClawRest, kClawOffset);
  Claw clawPitch    (kChClawPitch,  kClawRest, kClawOffset);
  clawRotate.setLimits   (kClawMinUs, kClawMaxUs);
  clawOpenServo.setLimits(kClawMinUs, kClawMaxUs);
  clawPitch.setLimits    (kClawMinUs, kClawMaxUs);

  stopThruster(Config::kFrontLeftHorizontal,  frontLeftHorizontal,  driver);
  stopThruster(Config::kFrontRightHorizontal, frontRightHorizontal, driver);
  stopThruster(Config::kRearLeftHorizontal,   rearLeftHorizontal,   driver);
  stopThruster(Config::kRearRightHorizontal,  rearRightHorizontal,  driver);
  stopThruster(Config::kLeftVertical,         leftVertical,         driver);
  stopThruster(Config::kRightVertical,        rightVertical,        driver);
  stopThruster(Config::kLeftVertical2,        leftVertical2,        driver);
  stopThruster(Config::kRightVertical2,       rightVertical2,       driver);
  centerClaw(Config::kClawRotate, clawRotate,     driver);
  centerClaw(Config::kClawOpen,   clawOpenServo,  driver);
  centerClaw(Config::kClawPitch,  clawPitch,      driver);

  std::cout << "Active channels set to neutral, waiting for ESCs to arm...\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(kArmDelayMs));
  std::cout << "ROV is ON\n";

  Thruster_Mixer mixer;
  PID yawPID  (0.02f, 0.0f, 0.01f, -1.0f, 1.0f);
  PID pitchPID(0.02f, 0.0f, 0.01f, -1.0f, 1.0f);
  PID rollPID (0.02f, 0.0f, 0.01f, -1.0f, 1.0f);

  auto lastTime = std::chrono::steady_clock::now();

  // Servo hold-to-move state
  float clawRotatePos = 0.0f;   // current position [-1, 1]
  float clawOpenPos   = 0.0f;
  float clawPitchPos  = 0.0f;
  float clawRotateDir = 1.0f;   // travel direction, flips at limits
  float clawOpenDir   = 1.0f;
  float clawPitchDir  = 1.0f;

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
      centerClaw(Config::kClawRotate, clawRotate,     driver);
      centerClaw(Config::kClawOpen,   clawOpenServo,  driver);
      centerClaw(Config::kClawPitch,  clawPitch,      driver);
      yawPID.reset();
      pitchPID.reset();
      rollPID.reset();
      // Reset servo state to center
      clawRotatePos = clawOpenPos = clawPitchPos = 0.0f;
      clawRotateDir = clawOpenDir = clawPitchDir = 1.0f;
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

    // ── Servo hold-to-move logic ──────────────────────────────────────
    // While button held: move in current direction each tick
    // On release: hold position (no update sent)
    // At limit: clamp and flip direction for next press

    // X — clawRotate
    if (input.clawRotate != 0.0f) {
      clawRotatePos += kClawStep * clawRotateDir;
      if (clawRotatePos >= 1.0f) {
        clawRotatePos = 1.0f;
        clawRotateDir = -1.0f;
      } else if (clawRotatePos <= -1.0f) {
        clawRotatePos = -1.0f;
        clawRotateDir = 1.0f;
      }
      setPosClaw(Config::kClawRotate, clawRotate, clawRotatePos, driver);
    }

    // Y — clawOpen
    if (input.clawOpen != 0.0f) {
      clawOpenPos += kClawStep * clawOpenDir;
      if (clawOpenPos >= 1.0f) {
        clawOpenPos = 1.0f;
        clawOpenDir = -1.0f;
      } else if (clawOpenPos <= -1.0f) {
        clawOpenPos = -1.0f;
        clawOpenDir = 1.0f;
      }
      setPosClaw(Config::kClawOpen, clawOpenServo, clawOpenPos, driver);
    }

    // B — clawPitch
    if (input.clawPitch != 0.0f) {
      clawPitchPos += kClawStep * clawPitchDir;
      if (clawPitchPos >= 1.0f) {
        clawPitchPos = 1.0f;
        clawPitchDir = -1.0f;
      } else if (clawPitchPos <= -1.0f) {
        clawPitchPos = -1.0f;
        clawPitchDir = 1.0f;
      }
      setPosClaw(Config::kClawPitch, clawPitch, clawPitchPos, driver);
    }

    // ── Thrusters ─────────────────────────────────────────────────────
    float measuredPitch = 0.0f;
    float measuredYaw   = 0.0f;
    float measuredRoll  = 0.0f;

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

    const Thruster_Outputs output =
        mixer.mix(input, yawPIDOutput, pitchPIDOutput, rollPIDOutput);

    setPowerThruster(Config::kFrontLeftHorizontal,  frontLeftHorizontal,  output.frontLeftHorizontal,  driver);
    setPowerThruster(Config::kFrontRightHorizontal, frontRightHorizontal, output.frontRightHorizontal, driver);
    setPowerThruster(Config::kRearLeftHorizontal,   rearLeftHorizontal,   output.rearLeftHorizontal,   driver);
    setPowerThruster(Config::kRearRightHorizontal,  rearRightHorizontal,  output.rearRightHorizontal,  driver);
    setPowerThruster(Config::kLeftVertical,         leftVertical,         output.leftVertical,         driver);
    setPowerThruster(Config::kRightVertical,        rightVertical,        output.rightVertical,        driver);
    setPowerThruster(Config::kLeftVertical2,        leftVertical2,        output.leftVertical2,        driver);
    setPowerThruster(Config::kRightVertical2,       rightVertical2,       output.rightVertical2,       driver);

    std::cout << "ROV running | ClawR=" << clawRotatePos
              << " ClawO=" << clawOpenPos
              << " ClawP=" << clawPitchPos
              << "    \r";
    std::cout.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // Safe shutdown
  stopThruster(Config::kFrontLeftHorizontal,  frontLeftHorizontal,  driver);
  stopThruster(Config::kFrontRightHorizontal, frontRightHorizontal, driver);
  stopThruster(Config::kRearLeftHorizontal,   rearLeftHorizontal,   driver);
  stopThruster(Config::kRearRightHorizontal,  rearRightHorizontal,  driver);
  stopThruster(Config::kLeftVertical,         leftVertical,         driver);
  stopThruster(Config::kRightVertical,        rightVertical,        driver);
  stopThruster(Config::kLeftVertical2,        leftVertical2,        driver);
  stopThruster(Config::kRightVertical2,       rightVertical2,       driver);
  centerClaw(Config::kClawRotate, clawRotate,     driver);
  centerClaw(Config::kClawOpen,   clawOpenServo,  driver);
  centerClaw(Config::kClawPitch,  clawPitch,      driver);
  net.detach();
  std::cout << "\nExiting safely.\n";
  return 0;
}
