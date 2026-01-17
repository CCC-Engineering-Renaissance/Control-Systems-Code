#include "Thruster.h"
#include "PiPCA9685/PCA9685.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
using std::clamp;    //Just because I can... (see result on line 22)

namespace {
  constexpr int MIN_PIN = 0;
  constexpr int MAX_PIN = 15;

  //  Driver Code is written as if PWM values are in ms so we need to convert the values as we write PWM
  inline double us_to_ms(int us) { 
    return static_cast<double>(us) / 1000.0; 
    }
} 

    // Helper checks and clamps
bool Thruster::isCorrectPin(int p) const{
  return (p >= 0 && p <= 15);
}
double clampPower(double power) const;
int Thruster::clampPWM(int pwm_us) const{
  int clamped = clamp()
}

