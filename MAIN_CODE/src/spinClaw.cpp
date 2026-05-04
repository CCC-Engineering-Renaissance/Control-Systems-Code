#include <iostream>
#include <string>
#include <unistd.h>
#include "PCA9685.h"
#include "Thruster.h"

int main() {
    std::cout << "Initializing Claw Controller..." << std::endl;

    try {
        // Correctly passing the path as a string literal
        PiPCA9685::PCA9685 driver("/dev/i2c-1", 0x40);
        
        // Set to standard servo frequency (50Hz)
        driver.set_pwm_freq(50);

        // Instantiate the Thruster (uses FIXED_PIN 12 by default in your class)
        Thruster claw;

        std::cout << "Starting test sequence: Power 0.5 for 2 seconds." << std::endl;

        // Set power to 50%
        claw.setPower(0.5, driver);
        
        // Hold for 2 seconds
        usleep(2000000); 

        // Stop the motor
        claw.stop(driver);
        std::cout << "Test complete. Claw stopped." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "CRITICAL ERROR: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
