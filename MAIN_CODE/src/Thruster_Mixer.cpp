#include "Thruster_Mixer.h"

#include <cmath>
#include <algorithm>

float Thruster_Mixer::clamp1(float x) {
  if (x >  1.0f) return  1.0f;
  if (x < -1.0f) return -1.0f;
  return x;
}

void Thruster_Mixer::normalize4(float &a, float &b, float &c, float &d) {
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

Thruster_Outputs Thruster_Mixer::mix(const POVState& input, 
                                     float Yaw_PID_Output,
                                     float Pitch_PID_Output,
                                     float Roll_PID_Output) const{

 // Pull out the motion commands and clamp them to [-1, 1]
  float forwardCommand  = Thruster_Mixer::clamp1(input.forward);
  float strafeCommand   = Thruster_Mixer::clamp1(input.strafe);
  float yawCommand      = Thruster_Mixer::clamp1(input.yaw);
  float verticalCommand = Thruster_Mixer::clamp1(input.vertical);
  float pitchCommand    = Thruster_Mixer::clamp1(input.pitch);
  float rollCommand     = Thruster_Mixer::clamp1(input.roll);

  /*   Thruster_Outputs struct used here replaces names like float frontLeftPower = forwardCommand +... with
        output.frontLeftHorizontal = forwardCommand +...*/

  Thruster_Outputs output{}; //"output" comes from here so we dont say Thruster_Outputs.(thrusterobject)
  
  float Total_Yaw = yawCommand + Yaw_PID_Output;

  // Mix forward/strafe/yaw into 4 horizontal thrusters | side note: yaw is rotation about z axis
  // Left-side strafe signs are flipped to match how the thrusters are actually wired/mounted;
  // if strafe comes out mirrored in the water, flip the strafe signs on the right pair instead.
  output.frontLeftHorizontal  = forwardCommand - strafeCommand + Total_Yaw;
  output.frontRightHorizontal = forwardCommand - strafeCommand - Total_Yaw;
  output.rearLeftHorizontal   = forwardCommand + strafeCommand + Total_Yaw;
  output.rearRightHorizontal  = forwardCommand + strafeCommand - Total_Yaw;


  Thruster_Mixer::normalize4(
    output.frontLeftHorizontal,
    output.frontRightHorizontal,
    output.rearLeftHorizontal,
    output.rearRightHorizontal);

  // Mix vertical/pitch/roll into 4 vertical thrusters 

  float Total_Pitch = pitchCommand + Pitch_PID_Output;
  float Total_Roll = rollCommand + Roll_PID_Output;

  output.leftVertical   = verticalCommand + Total_Pitch + Total_Roll;
  output.rightVertical  = verticalCommand + Total_Pitch - Total_Roll;
  output.leftVertical2  = verticalCommand - Total_Pitch + Total_Roll;
  output.rightVertical2 = verticalCommand - Total_Pitch - Total_Roll;

  Thruster_Mixer::normalize4(
    output.leftVertical,
    output.rightVertical,
    output.leftVertical2,
    output.rightVertical2);

    return output;

}
