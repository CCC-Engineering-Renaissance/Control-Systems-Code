#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <csignal>

<<<<<<< HEAD
#include "connection.h"       // For POVState input
#include "Thruster_Mixer.h"   // Mix joystick/POV commands to thruster outputs
#include "Thruster.h"         // Individual thruster abstraction
#include "PCA9685.h"          // PWM driver for thrusters
#include "IMU.h"              // IMU + complementary filter

volatile bool keepRunning = true;

// Signal handler to stop loop safely
void signalHandler(int) {
    keepRunning = false;
=======
constexpr int PORT = 5005;

using namespace std;
static float clamp1(float x) {
  if (x >  1.0f) return  1.0f;
  if (x < -1.0f) return -1.0f;
  return x;
>>>>>>> cf9d9829 (added codex fixes)
}


int main() {
    // ---- Setup signal handling to allow CTRL+C exit ----
    std::signal(SIGINT, signalHandler);

    // ---- Initialize PCA9685 PWM driver ----
    PiPCA9685::PCA9685 pwmDriver("/dev/i2c-1", 0x40); // default I2C bus and address
    pwmDriver.set_pwm_freq(200.0);                     // 200 Hz for ESCs (check your ESC specs)

    // ---- Initialize thrusters ----
    // Pins should match your physical PCA9685 wiring
    Thruster frontLeftH(0);
    Thruster frontRightH(1);
    Thruster rearLeftH(2);
    Thruster rearRightH(3);
    Thruster leftV(4);
    Thruster rightV(5);
    Thruster leftV2(6);
    Thruster rightV2(7);

    // ---- Initialize mixer ----
    Thruster_Mixer mixer;

    // ---- Initialize IMU ----
    IMU imuSensor;
    imuSensor.initialize(); // placeholder: sets up MPU6050, calibrates if needed

    // ---- Start UDP server in separate thread ----
    std::thread serverThread(server, 12345);
    serverThread.detach();

    // ---- Control loop ----
    const auto loopPeriod = std::chrono::milliseconds(20); // 50 Hz
    while (keepRunning) {
        auto startTime = std::chrono::steady_clock::now();

        // Get latest POVState
        POVState input = get_State();

        // Optional: Use IMU angles to stabilize or add PID correction
        imuSensor.update(); // Reads raw MPU6050, runs complementary filter
        // Example: input.pitch += imuSensor.getPitchCorrection();
        // Placeholder for PID integration
        
        float Yaw_PID_Output = Yaw_PID.update(yaw_Setpoint, yaw_Measured, dt);
        float Pitch_PID_Output = Pitch_PID.update(pitch_Setpoint, pitch_Measured, dt);
        float Roll_PID_Output = Roll_PID.update(roll_Setpoint, roll_Measured, dt);
        // Mix to thruster outputs
        Thruster_Outputs outputs = mixer.mix(input, Yaw_PID_Output, Pitch_PID_Output, Roll_PID_Output);

        // Send outputs to hardware
        frontLeftH.setPower(outputs.frontLeftHorizontal, pwmDriver);
        frontRightH.setPower(outputs.frontRightHorizontal, pwmDriver);
        rearLeftH.setPower(outputs.rearLeftHorizontal, pwmDriver);
        rearRightH.setPower(outputs.rearRightHorizontal, pwmDriver);

        leftV.setPower(outputs.leftVertical, pwmDriver);
        rightV.setPower(outputs.rightVertical, pwmDriver);
        leftV2.setPower(outputs.leftVertical2, pwmDriver);
        rightV2.setPower(outputs.rightVertical2, pwmDriver);

        // Optional: Print telemetry for debugging
        // std::cout << "FL: " << outputs.frontLeftHorizontal << ", FR: " << outputs.frontRightHorizontal << "\n";

        // Wait until next loop iteration
        auto elapsed = std::chrono::steady_clock::now() - startTime;
        if (elapsed < loopPeriod)
            std::this_thread::sleep_for(loopPeriod - elapsed);
    }

    // ---- Stop all thrusters before exit ----
    frontLeftH.stop(pwmDriver);
    frontRightH.stop(pwmDriver);
    rearLeftH.stop(pwmDriver);
    rearRightH.stop(pwmDriver);
    leftV.stop(pwmDriver);
    rightV.stop(pwmDriver);
    leftV2.stop(pwmDriver);
    rightV2.stop(pwmDriver);

    std::cout << "Exiting safely." << std::endl;
    return 0;
}

