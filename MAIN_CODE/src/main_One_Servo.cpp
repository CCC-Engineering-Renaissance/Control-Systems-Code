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

  constexpr int kChClaw    = 8;
  //constexpr int kChClaw2   = 9;

  constexpr int kClawRest  = 500;
  constexpr int kClawOffset = 50000;
  constexpr int kClawMinUs  = 500;
  constexpr int kClawMaxUs  = 2500;
  void signalHandler(int) { keepRunning = 0; }
}

int main() {
  std::signal(SIGINT, signalHandler);
  std::cout << "Starting dual servo test...\n";

  std::thread net([] {
    try { server(kPort); }
    catch (const std::exception& e) {
      std::cerr << "Network thread exception: " << e.what() << "\n";
    }
  });

  PiPCA9685::PCA9685 driver("/dev/i2c-1", 0x40);
  driver.set_pwm_freq(50.0);

  Claw servo1(kChClaw,  kClawRest, kClawOffset);
  //Claw servo2(kChClaw2, kClawRest, kClawOffset);
  servo1.setLimits(kClawMinUs, kClawMaxUs);
  //servo2.setLimits(kClawMinUs, kClawMaxUs);
  servo1.center(driver);
 // servo2.center(driver);

  std::cout << "Servos centered, waiting for ESC arm...\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(kArmDelayMs));
  std::cout << "Ready\n";

  while (keepRunning) {
    if (!is_Fresh(kStalePacketMs)) {
      servo1.center(driver);
      //servo2.center(driver);
      std::cout << "Waiting for packets...\r";
      std::cout.flush();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    const POVState input = get_State();
    servo1.setPosition(input.clawPitch,  driver);   // right joystick Y
    //servo2.setPosition(input.clawRotate, driver);   // left joystick Y

    std::cout << "Servo1 (RJY): " << input.clawPitch
              << "  Servo2 (LJY): " << input.clawRotate
              << "    \r";
    std::cout.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  servo1.center(driver);
  //servo2.center(driver);
  net.detach();
  std::cout << "\nExiting safely.\n";
  return 0;
}
