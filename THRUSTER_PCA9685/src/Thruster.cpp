#include "Thruster.h"
#include "PiPCA9685/PCA9685.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
  constexpr int MIN_PIN = 0;
  constexpr int MAX_PIN = 15;

//  Driver Code is written as if PWM values are in ms so we need to convert the values as we write PWM
inline double us_to_ms(int us) { 
  return static_cast<double>(us) / 1000.0; 
  }
} 
