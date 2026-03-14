
#include "I2CPeripheral.h"
#include "PCA9685.h"
#include "Thruster.h"
#include "connection.h"
//#include <Eigen/Dense>
//#include <Eigen/QR>
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <algorithm>

constexpr int PORT = 5005;

using namespace std;
static float clamp1(float x) {
  if (x >  1.0f) return  1.0f;
  if (x < -1.0f) return -1.0f;
  return x;
}

// If any of the 4 values exceed magnitude 1, scale all 4 down proportionally.
// This preserves direction (mix ratio) while preventing saturation/clipping.
static void normalize4(float &a, float &b, float &c, float &d) {
  float m = std::max({std::abs(a), std::abs(b), std::abs(c), std::abs(d)});
  if (m > 1.0f) {
    a /= m; b /= m; c /= m; d /= m;
  }
}

int main() {

  // Run the UDP receiver in the background.
  // server(PORT) blocks forever, so it must run in a separate thread.
  std::thread net([&] { server(PORT); });

  // Create the PCA9685 driver object:
  // - "/dev/i2c-1" is the standard Raspberry Pi I2C bus
  // - 0x40 is the default PCA9685 address (change if your board uses different address)
  PiPCA9685::PCA9685 driver("/dev/i2c-1", 0x40);

  // Set PWM frequency for ESCs.
  // Most ESCs expect ~50 Hz for "servo PWM" style pulses.
  driver.set_pwm_freq(50);

  // PCA9685 has 16 channels: 0..15.
  // Update these numbers to match acutal physical wiring.
  constexpr int THRUSTER_TEST_CHANNEL  = 0;

  // Thruster(pin) sets default rest=1500us, offset=400us, min=1100us, max=1900us

  Thruster Test_Thruster(THRUSTER_TEST_CHANNEL);
  
  /*
  Thruster frontRightHorizontal(FRONT_RIGHT_HORIZONTAL_CHANNEL);
  Thruster rearLeftHorizontal(REAR_LEFT_HORIZONTAL_CHANNEL);
  Thruster rearRightHorizontal(REAR_RIGHT_HORIZONTAL_CHANNEL);
  Thruster leftVertical(LEFT_VERTICAL_CHANNEL);
  Thruster rightVertical(RIGHT_VERTICAL_CHANNEL);
  Thruster leftVertical2(LEFT_VERTICAL_CHANNEL_2);
  Thruster rightVertical2(RIGHT_VERTICAL_CHANNEL_2);
*/
  cout << "Listening on UDP port " << PORT << "...\n";




  while (true) {

   

    // is_Fresh(250) is true if we got a packet within last 250ms.
    // If Python dies or WiFi drops, stop motors.
    //
    if (!is_Fresh(250)) {

      Test_Thruster.stop(driver);
      /*
      frontRightHorizontal.stop(driver);
      rearLeftHorizontal.stop(driver);
      rearRightHorizontal.stop(driver);
      leftVertical.stop(driver);
      rightVertical.stop(driver);
      leftVertical2.stop(driver);
      rightVertical2.stop(driver);
      */
      cout << "STALE: stop\r";
      cout.flush();

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    // Read the latest decoded controller state (thread-safe snapshot)

    POVState s = get_State();

                                                    

    float test_command  = clamp1(s.forward);
   
    /*float strafeCommand   = clamp1(s.strafe);       // pick test command
    float yawCommand      = clamp1(s.yaw);
    float verticalCommand = clamp1(s.vertical);
    float pitchCommand    = clamp1(s.pitch);
    float rollCommand     = clamp1(s.roll);
*/
    // Mix forward/strafe/yaw into 4 horizontal thrusters
/*
    float frontLeftPower  = forwardCommand + strafeCommand + yawCommand;
    float frontRightPower = forwardCommand - strafeCommand - yawCommand;
    float rearLeftPower   = forwardCommand - strafeCommand + yawCommand;
    float rearRightPower  = forwardCommand + strafeCommand - yawCommand;
*/
    // Normalize if any exceed magnitude 1
//    normalize4(frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);

    // Send power to horizontal thrusters
    Test_Thruster.setPower(test_command, driver);
/*
    frontRightHorizontal.setPower(frontRightPower, driver);
    rearLeftHorizontal.setPower(rearLeftPower, driver);
    rearRightHorizontal.setPower(rearRightPower, driver);

  float leftVerticalPower   = verticalCommand + pitchCommand + rollCommand;
  float rightVerticalPower  = verticalCommand + pitchCommand - rollCommand;
  float leftVertical2Power  = verticalCommand - pitchCommand + rollCommand;
  float rightVertical2Power = verticalCommand - pitchCommand - rollCommand;

  // Normalize 
  normalize4(leftVerticalPower, rightVerticalPower, leftVertical2Power, rightVertical2Power);

  // Send power to verical thrusters
  leftVertical.setPower(leftVerticalPower, driver);
  rightVertical.setPower(rightVerticalPower, driver);
  leftVertical2.setPower(leftVertical2Power, driver);
  rightVertical2.setPower(rightVertical2Power, driver);
*/

    // Debug print: show the incoming command values and final thruster outputs

        
    cout << "command= " << test_command 
    //<< "up=" << verticalCommand
    //<< " pitch=" << pitchCommand
    //<< " roll=" << rollCommand
    //<< " | LVert=" << leftVerticalPower
    //<< " RVert=" << rightVerticalPower
    //<< " LVert2=" << leftVertical2Power
  //  << " RVert2=" << rightVertical2Power
    << "     \r";
    cout.flush();

    this_thread::sleep_for(chrono::milliseconds(50));
  }

  net.join();
  return 0;
}
