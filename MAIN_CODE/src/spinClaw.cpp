#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>
#include "PCA9685.h"
#include "Thruster.h"

static constexpr int kDefaultChannel = 8;  // change or pass as argv[1]

int main(int argc, char** argv) {
    const int channel = (argc > 1) ? std::atoi(argv[1]) : kDefaultChannel;

    std::cout << "spinClaw: testing channel " << channel << "\n";

    try {
        PiPCA9685::PCA9685 driver("/dev/i2c-1", 0x40);
        driver.set_pwm_freq(50.0);

        Thruster claw(channel);

        // Send neutral so the ESC can arm before accepting commands
        claw.stop(driver);
        std::cout << "Neutral sent. Waiting for ESC to arm...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        std::cout << "Running at 0.5 power for 2 seconds...\n";
        claw.setPower(0.5, driver);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        claw.stop(driver);
        std::cout << "Done.\n";

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
