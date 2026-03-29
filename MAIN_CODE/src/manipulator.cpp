#include "../include/Claw.h"
#include "../include/PCA9685.h"
#include "../include/connection.h"
#include <iostream>
#include <thread>
#include <chrono>
using namespace std;

constexpr int PORT = 5005;

// PCA9685, change pins
constexpr int CH_ROTATE = 0;
constexpr int CH_OPEN   = 1;
constexpr int CH_PITCH  = 2;
constexpr int CH_OPEN2  = 3;

int main() {
    std::thread net([&] { server(PORT); });

    PiPCA9685::PCA9685 driver("/dev/i2c-1", 0x40);
    driver.set_pwm_freq(50);

    // Adjust rest, offset, limits for servo after test
    Claw rotate(CH_ROTATE,1500,1000);
    Claw open(CH_OPEN,1500,1000);
    Claw pitch(CH_PITCH,1500,1000);
    Claw open2(CH_OPEN2,1500,1000);

    rotate.setLimits(500,2500);
    open.setLimits(500,2500);

    pitch.setLimits(500, 2500);
    open2.setLimits(500, 2500);

    Claw* servos[] = { &rotate, &open, &pitch, &open2 };
    constexpr int NUM_SERVOS = 4;

    cout << "Manipulator running on UDP port " << PORT
    << "  (" << NUM_SERVOS << " servos)\n";

    while (true) {
        if (!is_Fresh(250)) {
            for (int i = 0; i < NUM_SERVOS; i++)
                servos[i]->center(driver);

            cout << "STALE: centering all servos\r";
            cout.flush();
            this_thread::sleep_for(chrono::milliseconds(50));
            continue;
        }

        POVState s = get_State();

        rotate.setPosition(s.clawRotate, driver);
        open.setPosition(s.clawOpen,     driver);
        pitch.setPosition(s.clawPitch,   driver);
        open2.setPosition(s.claw1Open,   driver);

        cout << "rot:" << rotate.getPWM()
        << "  opn:" << open.getPWM()
        << "  pit:" << pitch.getPWM()
        << "  op2:" << open2.getPWM() << " us   \r";
        cout.flush();

        this_thread::sleep_for(chrono::milliseconds(50));
    }

    net.join();
    return 0;
}
