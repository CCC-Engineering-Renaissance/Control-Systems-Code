#pragma once 

#include "connection.h"

// inside conection.h we have made POVState{} which we can use as our "inputs" and blend them in the mix method

/* The Thruster_Outputs struct is the same as what we used to have in main 
   where we initialized each thruster object. 
   Now it is involved with mixer so that the outputs arent just manual intent */
struct Thruster_Outputs {

  float frontLeftHorizontal;
  float frontRightHorizontal;
  float rearLeftHorizontal;
  float rearRightHorizontal;
  float leftVertical;
  float rightVertical;
  float leftVertical2;
  float rightVertical2;
};

// For now (3/21/26) I am tranferring all logic pertaining to thruster movement from main to this class
// Later comes the addition of PID contribution to the mix, which is less than a week away... SO MUCH HOMEWORK nah jokes all jokes just jokes. API King here just joshin
class Thruster_Mixer {

private:
  static float clamp1(float x);
  static void normalize4(float& a, float& b, float& c, float& d);

public:
  Thruster_Outputs mix(const POVState& s) const;
};
