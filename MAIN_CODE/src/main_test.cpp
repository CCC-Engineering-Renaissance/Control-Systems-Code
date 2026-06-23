#include "I2CPeripheral.h"
#include "PCA9685.h"
#include "Thruster.h"
#include "Thruster_Mixer.h"
#include "connection.h"
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

using namespace std;
/*static float clamp1(float x) {
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
*/ // commented out lines 14 - 34 to test Thruster_Mixer class
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
  constexpr int FRONT_LEFT_HORIZONTAL_CHANNEL  = 0;
  constexpr int FRONT_RIGHT_HORIZONTAL_CHANNEL = 1;
  constexpr int REAR_LEFT_HORIZONTAL_CHANNEL   = 2;
  constexpr int REAR_RIGHT_HORIZONTAL_CHANNEL  = 3;
  constexpr int LEFT_VERTICAL_CHANNEL          = 4;
  constexpr int RIGHT_VERTICAL_CHANNEL         = 5;
  constexpr int LEFT_VERTICAL_CHANNEL_2        = 6;
  constexpr int RIGHT_VERTICAL_CHANNEL_2       = 7;

  // Thruster(pin) sets default rest=1500us, offset=400us, min=1100us, max=1900us

  Thruster frontLeftHorizontal(FRONT_LEFT_HORIZONTAL_CHANNEL);
  Thruster frontRightHorizontal(FRONT_RIGHT_HORIZONTAL_CHANNEL);
  Thruster rearLeftHorizontal(REAR_LEFT_HORIZONTAL_CHANNEL);
  Thruster rearRightHorizontal(REAR_RIGHT_HORIZONTAL_CHANNEL);
  Thruster leftVertical(LEFT_VERTICAL_CHANNEL);
  Thruster rightVertical(RIGHT_VERTICAL_CHANNEL);
  Thruster leftVertical2(LEFT_VERTICAL_CHANNEL_2);
  Thruster rightVertical2(RIGHT_VERTICAL_CHANNEL_2);

  Thruster_Mixer mixer; // create mixer object to use for mixing each thruster object

  cout << "Listening on UDP port " << PORT << "...\n";

// Control loop
  while (true) {
   
    // Safety stop: if packets are stale, stop everything

    // is_Fresh(250) is true if we got a packet within last 250ms.
    // If Python dies or WiFi drops, stop motors.
    //
    if (!is_Fresh(250)) {

      frontLeftHorizontal.stop(driver);
      frontRightHorizontal.stop(driver);
      rearLeftHorizontal.stop(driver);
      rearRightHorizontal.stop(driver);
      leftVertical.stop(driver);
      rightVertical.stop(driver);
      leftVertical2.stop(driver);
      rightVertical2.stop(driver);
      
      cout << "STALE: stop\r";
      cout.flush();

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    // Read the latest decoded controller state (thread-safe snapshot)

    POVState s = get_State();
    
    // mix thruster math for every object in "mix" method in Thruster_Mixer.cpp
    Thruster_Outputs output = mixer.mix(input);

    // Pull out the motion commands and clamp them to [-1, 1]
    /*
    float forwardCommand  = clamp1(s.forward);
    float strafeCommand   = clamp1(s.strafe);
    float yawCommand      = clamp1(s.yaw);
    float verticalCommand = clamp1(s.vertical);
    float pitchCommand    = clamp1(s.pitch);
    float rollCommand     = clamp1(s.roll);

    // Mix forward/strafe/yaw into 4 horizontal thrusters

    float frontLeftPower  = forwardCommand + strafeCommand + yawCommand;
    float frontRightPower = forwardCommand - strafeCommand - yawCommand;
    float rearLeftPower   = forwardCommand - strafeCommand + yawCommand;
    float rearRightPower  = forwardCommand + strafeCommand - yawCommand;

    // Normalize if any exceed magnitude 1
    normalize4(frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);
    
    // Send power to horizontal thrusters
    frontLeftHorizontal.setPower(frontLeftPower, driver);
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

    // if this works you can delete the commented out logic 

    //send power to vertical thrusters 
  frontLeftHorizontal.setPower(output.frontLeftHorizontal, driver);
  frontRightHorizontal.setPower(output.frontRightHorizontal, driver);
  rearLeftHorizontal.setPower(output.rearLeftHorizontal, driver);
  rearRightHorizontal.setPower(output.rearRightHorizontal, driver);

  leftVertical.setPower(output.leftVertical, driver);
  rightVertical.setPower(output.rightVertical, driver);
  leftVertical2.setPower(output.leftVertical2, driver);
  rightVertical2.setPower(output.rightVertical2, driver);

    // Debug print: show the incoming command values and final thruster outputs
  
    /*
    cout
    << "input_up=" << verticalCommand
    << " pitch=" << pitchCommand
    << " roll=" << rollCommand
    << " | LVert=" << leftVerticalPower
    << " RVert=" << rightVerticalPower
    << " LVert2=" << leftVertical2Power
    << " RVert2=" << rightVertical2Power
    << "     \r";
    cout.flush();
  */
    cout
    << "fwd=" << input.forward
    << " strafe=" << input.strafe
    << " yaw=" << input.yaw
    << " | FLeftHor=" << output.frontLeftHorizontal
    << " FRightHor=" << output.frontRightHorizontal
    << " RLeftHor=" << output.rearLeftHorizontal
    << " RRightHor=" << output.rearRightHorizontal
    << " || up=" << input.vertical
    << " pitch=" << input.pitch
    << " roll=" << input.roll
    << " | LeftVert=" << output.leftVertical
    << " RightVert=" << output.rightVertical
    << " LeftVert2=" << output.leftVertical2
    << " RightVert2=" << output.rightVertical2
    << "     \r";
cout.flush();
    this_thread::sleep_for(chrono::milliseconds(50));
  }

  net.join();
  return 0;
}
