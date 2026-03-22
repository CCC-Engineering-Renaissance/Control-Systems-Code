#include "Thruster_Mixer.h"

#include <cmath>
#include <algorithm>

// Below are the lines in main from 19-32
float clamp1(float x) {
  if (x >  1.0f) return  1.0f;
  if (x < -1.0f) return -1.0f;
  return x;
}

void normalize4(float &a, float &b, float &c, float &d) {
  float m = std::max({std::abs(a), std::abs(b), std::abs(c), std::abs(d)});
  if (m > 1.0f) {
    a /= m;
    b /= m;
    c /= m;
    d /= m;
  }
}

// Below is logic written in main as of 3/21/26 from line 109-136
// At some point during the following 6 days it will be altered to include PID

Thruster_Outputs Thruster_Mixer::mix(const POVState& input) const{

 // Pull out the motion commands and clamp them to [-1, 1]
  float forwardCommand  = clamp1(input.forward);
  float strafeCommand   = clamp1(input.strafe);
  float yawCommand      = clamp1(input.yaw);
  float verticalCommand = clamp1(input.vertical);
  float pitchCommand    = clamp1(input.pitch);
  float rollCommand     = clamp1(input.roll);

  /*   Thruster_Outputs struct used here replaces names like float frontLeftPower = forwardCommand +... with
        output.frontLeftHorizontal = forwardCommand +...*/

  Thruster_Outputs output{} //"output" comes from here so we dont say Thruster_Outputs.(thrusterobject)

  // Mix forward/strafe/yaw into 4 horizontal thrusters | side note: yaw is rotation about z axis
  output.frontLeftHorizontal  = forwardCommand + strafeCommand + yawCommand;
  output.frontRightHorizontal = forwardCommand - strafeCommand - yawCommand;
  output.rearLeftHorizontal   = forwardCommand - strafeCommand + yawCommand;
  output.rearRightHorizontal  = forwardCommand + strafeCommand - yawCommand;


  normalize4(
    output.frontLeftHorizontal,
    output.frontRightHorizontal,
    output.rearLeftHorizontal,
    output.rearRightHorizontal);

  // Mix vertical/pitch/roll into 4 vertical thrusters 
  output.leftVertical   = verticalCommand + pitchCommand + rollCommand;
  output.rightVertical  = verticalCommand + pitchCommand - rollCommand;
  output.leftVertical2  = verticalCommand - pitchCommand + rollCommand;
  output.rightVertical2 = verticalCommand - pitchCommand - rollCommand;

  normalize4(
    output.leftVertical,
    output.rightVertical,
    output.leftVertical2,
    output.rightVertical2);

    return out;

}
