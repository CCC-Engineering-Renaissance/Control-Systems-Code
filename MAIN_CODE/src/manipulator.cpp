#include "../include/Claw.h"
#include "../include/PCA9685.h"
#include "../include/connection.h"
#include <iostream>
#include <thread>
#include <chrono>
using namespace std;

constexpr int PORT = 5005;


// this is a test, I used AI to create this.
int main() {
    std::thread net([&] { server(PORT); });

    PiPCA9685::PCA9685 driver("/dev/i2c-1", 0x40);
    driver.set_pwm_freq(50);

    constexpr int INJORA_CHANNEL = 0;


    Claw injoraServo(INJORA_CHANNEL, 1500, 1000);

    injoraServo.setLimits(500, 2500);

    cout << "Manipulator test running on UDP port " << PORT << "...\n";

    while (true) {
        if (!is_Fresh(250)) {
            injoraServo.center(driver);
            cout << "STALE: centering servo\r";
            cout.flush();
            this_thread::sleep_for(chrono::milliseconds(50));
            continue;
        }

        POVState s = get_State();


        double command = s.clawRotate;

        injoraServo.setPosition(command, driver);

        cout << "Input: " << command
        << " | Servo PWM: " << injoraServo.getPWM() << " us      \r";
        cout.flush();

        this_thread::sleep_for(chrono::milliseconds(50));
    }

    net.join();
    return 0;
}
