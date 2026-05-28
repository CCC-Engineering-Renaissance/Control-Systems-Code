#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "PCA9685.h"
#include "Thruster.h"
#include "Thruster_Mixer.h"
#include "Claw.h"
#include "connection.h"
#include "PID.h"
#include "IMU.h"
#include "Depth_Sensor.h"

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
  constexpr bool kClawBrushless        = true;   // brushless motor on ch10
  constexpr bool kPID                  = true;
  constexpr bool kIMU                  = true;
  constexpr bool kDepthSensor          = true;
}

namespace {
  volatile std::sig_atomic_t keepRunning = 1;
  constexpr unsigned short kPort        = 5005;
  constexpr unsigned short kTelemPort   = 5006;
  constexpr const char*    kTopsideIP   = "192.168.2.1";   // change to your topside PC IP
  constexpr auto kTelemInterval         = std::chrono::milliseconds(50);   // 20 Hz
  constexpr int   kStalePacketMs        = 500;
  constexpr float kMaxDt                = 0.1f;
  constexpr int   kArmDelayMs           = 3000;

  constexpr int   kChClawRotate     = 8;
  constexpr int   kChClawOpen       = 9;
  constexpr int   kChClawBrushless  = 10;  // third claw — brushless motor
  constexpr int   kClawOffset   = 364;
  constexpr int   kClawRest     = 1500 - (0 * kClawOffset);
  constexpr int   kClawMinUs    = kClawOffset;
  constexpr int   kClawMaxUs    = 1500 + kClawOffset;
  constexpr float kClawSpeed    = 1.5f;
  constexpr float kMaxThrustCoeff = 0.8f;
  constexpr float kSlowModePercent = 0.2f;

  void signalHandler(int) { keepRunning = 0; }

  std::atomic<float> gDepthMeters{0.0f};
  std::atomic<float> gTempC{0.0f};
  std::atomic<bool>  gDepthReady{false};
}

static void depthSensorThread(DepthSensor& sensor) {
  if (!sensor.init()) {
    std::cerr << "Depth sensor init failed — depth readings unavailable\n";
    return;
  }
  // Zero depth at the surface using the first successful read
  if (sensor.read()) {
    sensor.setSurfacePressure(sensor.getPressureMbar());
    std::cout << "Depth sensor zeroed at " << sensor.getPressureMbar() << " mbar\n";
  }
  while (keepRunning) {
    if (sensor.read()) {
      gDepthMeters.store(sensor.getDepthMeters(),   std::memory_order_relaxed);
      gTempC      .store(sensor.getTemperatureC(),   std::memory_order_relaxed);
      gDepthReady .store(true,                       std::memory_order_release);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
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
  std::cout << "  SpinClaw(ch8) : " << (Config::kClawRotate            ? "ON" : "OFF") << "\n";
  std::cout << "  Servo2  (ch9) : " << (Config::kClawOpen             ? "ON" : "OFF") << "\n";
  std::cout << "  Brushless(ch10): " << (Config::kClawBrushless        ? "ON" : "OFF") << "\n";
  std::cout << "  PID         : " << (Config::kPID                  ? "ON" : "OFF") << "\n";
  std::cout << "  IMU         : " << (Config::kIMU                  ? "ON" : "OFF") << "\n";
  std::cout << "  DepthSensor : " << (Config::kDepthSensor          ? "ON" : "OFF") << "\n";
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

  // ── Telemetry socket: ROV → topside PC ───────────────────────────────────
  int telemSock = socket(AF_INET, SOCK_DGRAM, 0);
  if (telemSock < 0) {
    std::cerr << "Failed to create telemetry socket\n";
  } else {
    std::cout << "Telemetry socket created — sending to "
              << kTopsideIP << ":" << kTelemPort << "\n";
  }
  sockaddr_in telemAddr{};
  telemAddr.sin_family = AF_INET;
  telemAddr.sin_port   = htons(kTelemPort);
  inet_pton(AF_INET, kTopsideIP, &telemAddr.sin_addr);

  auto lastTelemSend = std::chrono::steady_clock::now();

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

  Claw clawSpin(kChClawRotate, kClawRest, kClawOffset);  // ch8 — servo, Y=open / B=close
  clawSpin.setLimits(kClawMinUs, kClawMaxUs);
  Thruster clawBrushless(kChClawBrushless);  // ch10 — brushless claw motor

  Claw clawOpen(kChClawOpen, kClawRest, kClawOffset);
  clawOpen.setLimits(kClawMinUs, kClawMaxUs);

  // ── SafeStop guard ───────────────────────────────────────────────────────
  struct SafeStop {
    PiPCA9685::PCA9685& drv;
    Thruster &flh, &frh, &rlh, &rrh, &lv, &rv, &lv2, &rv2, &cb;
    Claw &cs, &cp;
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
      safe([&]{ stopThruster(Config::kClawBrushless,        cb,   drv); });
      safe([&]{ centerClaw(Config::kClawRotate, cs, drv); });
      safe([&]{ centerClaw(Config::kClawOpen,   cp, drv); });
    }
  } guard{driver,
    frontLeftHorizontal, frontRightHorizontal, rearLeftHorizontal, rearRightHorizontal,
    leftVertical, rightVertical, leftVertical2, rightVertical2,
    clawBrushless, clawSpin, clawOpen};
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
  stopThruster(Config::kClawRotate, clawSpin,  driver);
  centerClaw(Config::kClawOpen,   clawOpen,  driver);
  std::cout << "  NEUTRAL PWM value: " << frontLeftHorizontal.getPWM() << "us\n";
  std::cout << "CALIBRATION COMPLETE - waiting for arm...\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(kArmDelayMs));
 */

  // ch8 is now a servo — no ESC arming needed; center it at startup
  if (Config::kClawRotate) {
    clawSpin.center(driver);
    std::cout << "Spin servo (ch8) centred at startup.\n";
  }

  // Arm brushless claw ESC on ch10
  if (Config::kClawBrushless) {
    clawBrushless.stop(driver);
    std::cout << "Brushless claw (ch10) ESC neutral sent. Waiting for ESC to arm...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    std::cout << "Brushless claw (ch10): armed and ready.\n";
  }

  std::cout << "ROV is ON\n";

  std::cout << "── Network interfaces ───────────────\n";
  std::system("ip -brief addr show | grep -E 'eth0|wlan0|end0' || echo '  No eth/wlan interfaces found'");
  std::cout << "── UDP socket bound on port " << kPort << " ──\n";
  std::cout << "── Telemetry sending on port " << kTelemPort << " ──\n";
  std::cout << "─────────────────────────────────────\n";

  Thruster_Mixer mixer;
  PID yawPID  (0.02f, 0.0f, 0.01f, -1.0f, 1.0f);
  PID pitchPID(0.02f, 0.0f, 0.01f, -1.0f, 1.0f);
  PID rollPID (0.02f, 0.0f, 0.01f, -1.0f, 1.0f);
  PID depthPID(0.50f, 0.0f, 0.10f, -1.0f, 1.0f);

  std::unique_ptr<IMU> imu;
  if (Config::kIMU) {
    imu = std::make_unique<IMU>();
    std::cout << "IMU initialized at 0x68\n";
  }

  DepthSensor depthSensor;
  std::thread depthThread;
  if (Config::kDepthSensor) {
    depthThread = std::thread(depthSensorThread, std::ref(depthSensor));
    std::cout << "Depth sensor thread started on /dev/i2c-1 at 0x76\n";
  }

  float clawSpinPos = 0.0f;   // ch8 servo position
  float clawOpenPos = 0.0f;   // ch9 servo position

  float depthSetpoint = 0.0f;   // locked when ALS first activates
  bool  prevAls       = false;  // edge-detect ALS toggle

  auto lastTime = std::chrono::steady_clock::now();
  bool wasWaiting = true;

  try {
  while (keepRunning) {
    setPosClaw(Config::kClawRotate, clawSpin,  clawSpinPos,  driver);
    setPosClaw(Config::kClawOpen,   clawOpen,  clawOpenPos,  driver);

    if (!is_Fresh(kStalePacketMs)) {
      stopThruster(Config::kFrontLeftHorizontal,  frontLeftHorizontal,  driver);
      stopThruster(Config::kFrontRightHorizontal, frontRightHorizontal, driver);
      stopThruster(Config::kRearLeftHorizontal,   rearLeftHorizontal,   driver);
      stopThruster(Config::kRearRightHorizontal,  rearRightHorizontal,  driver);
      stopThruster(Config::kLeftVertical,         leftVertical,         driver);
      stopThruster(Config::kRightVertical,        rightVertical,        driver);
      stopThruster(Config::kLeftVertical2,        leftVertical2,        driver);
      stopThruster(Config::kRightVertical2,       rightVertical2,       driver);
      stopThruster(Config::kClawBrushless,        clawBrushless,        driver);
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

    // ALS rising edge — latch depth setpoint once when ALS first turns on
    if (Config::kDepthSensor && input.als && !prevAls &&
        gDepthReady.load(std::memory_order_acquire)) {
      depthSetpoint = gDepthMeters.load(std::memory_order_relaxed);
      std::cout << "ALS ON  — depth setpoint locked at "
                << depthSetpoint << " m\n";
    }
    if (!input.als && prevAls) {
      std::cout << "ALS OFF — returning to manual control\n";
    }
    prevAls = input.als;

    clawSpinPos = std::clamp(clawSpinPos + input.clawRotate * kClawSpeed * dt, -1.0f, 1.0f);
    setPowerThruster(Config::kClawBrushless, clawBrushless,  input.clawBrushless * kMaxThrustCoeff, driver);
    clawOpenPos = std::clamp(clawOpenPos + input.clawOpen * kClawSpeed * dt, -1.0f, 1.0f);

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
      depthPID.reset();
    }

    POVState mixInput = input;
    mixInput.forward = -mixInput.forward;
    mixInput.yaw = -mixInput.yaw;
    if (Config::kPID && input.als) {
      mixInput.yaw      = 0.0f;   // PID drives yaw
      mixInput.pitch    = 0.0f;   // PID drives pitch
      mixInput.roll     = 0.0f;   // PID drives roll
      // Depth hold: PID output feeds vertical; override manual vertical
      float currentDepth    = gDepthMeters.load(std::memory_order_relaxed);
      float depthOutput     = depthPID.update(depthSetpoint, currentDepth, dt);
      mixInput.vertical     = depthOutput;
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

    // ── Telemetry send (20 Hz) ───────────────────────────────────────────
    auto tNow = std::chrono::steady_clock::now();
    if (telemSock >= 0 && tNow - lastTelemSend >= kTelemInterval) {
      float depth = gDepthMeters.load(std::memory_order_relaxed);
      float temp  = gTempC.load(std::memory_order_relaxed);

      char buf[512];
      int n = snprintf(buf, sizeof(buf),
          "{\"depth\":%.3f,\"temp\":%.2f,"
          "\"pitch\":%.2f,\"yaw\":%.2f,\"roll\":%.2f,"
          "\"depth_setpoint\":%.3f,\"als\":%s,"
          "\"yaw_kp\":%.4f,\"yaw_ki\":%.4f,\"yaw_kd\":%.4f,"
          "\"pitch_kp\":%.4f,\"pitch_ki\":%.4f,\"pitch_kd\":%.4f,"
          "\"roll_kp\":%.4f,\"roll_ki\":%.4f,\"roll_kd\":%.4f,"
          "\"depth_kp\":%.4f,\"depth_ki\":%.4f,\"depth_kd\":%.4f}",
          depth, temp,
          measuredPitch, measuredYaw, measuredRoll,
          depthSetpoint,
          input.als ? "true" : "false",
          yawPID.getKp(),   yawPID.getKi(),   yawPID.getKd(),
          pitchPID.getKp(), pitchPID.getKi(), pitchPID.getKd(),
          rollPID.getKp(),  rollPID.getKi(),  rollPID.getKd(),
          depthPID.getKp(), depthPID.getKi(), depthPID.getKd());

      if (n > 0) {
        sendto(telemSock, buf, n, 0,
               (sockaddr*)&telemAddr, sizeof(telemAddr));
      }
      lastTelemSend = tNow;
    }

    // Console print (depth sensor)
    if (Config::kDepthSensor && gDepthReady.load(std::memory_order_acquire)) {
      std::cout << "Depth: " << gDepthMeters.load(std::memory_order_relaxed) << " m"
                << "  Temp: " << gTempC.load(std::memory_order_relaxed) << " C\n";
      gDepthReady.store(false, std::memory_order_relaxed);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  } catch (const std::exception& e) {
    std::cerr << "\nControl loop exception: " << e.what() << "\n";
  }

  stopServer();
  net.join();
  if (Config::kDepthSensor && depthThread.joinable()) depthThread.join();
  if (telemSock >= 0) close(telemSock);
  std::cout << "\nExiting safely.\n";
  return 0;
}
