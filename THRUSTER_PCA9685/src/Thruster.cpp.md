# Thruster.cpp design notes

## Background on Thruster.cpp
- Responsible for converting control inputs into PWM signals for ESCS
- Ensures thrusters operate within safe ranges preventing hardware malfunction
- Creates software abstraction layer that allows for high-level code to remain independent from hardware
- This file will predominantly be working with handling ROV Thrusters via PCA9685 PWM signals
- Will also be converting abstract commands into hardware-safe PWM signals


### 1. Includes

```cpp
#include "Thruster.h"
#include "PiPCA9685/PCA9685.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
using std::clamp;

* "Thruster.h" -> Declares the Thruster class, its constructors, and public methods
* "PiPCA9685/PCA9685.h" -> Provides the interface that allows for communication amongst the PCA9685 PWM controller hardware
* <algorithm> -> Contains std::clamp(), used to restrict values like power or PWM within safe ranges
* <cmath> -> Allows for math functions such as std::lround() & std::abs() to be used. Also key for converting values
* <stdexcept> -> Utilizes std::invalid_argument to signal invalid inputs safely
* using std::clamp; -> Lets user call clamp() without having to incorporate std:: each time, reducing redundancy



### 2. Internal Constants and functions

namespace {
  constexpr int MIN_PIN = 0;
  constexpr int MAX_PIN = 15;

  inline double us_to_ms(int us) { 
    return static_cast<double>(us) / 1000.0; 
  }
}


* We begin by creating an anonymous namespace. The reason for using an anonymous namespace is that it better protects code from unwanted variable changes and to further protect constraints byt limiting access to code
* In the anonymous namespace we create two constant expressions that express the minimum and maximum pin values that any hardware of the thruster can be connected. The variables in regards to the constant expressions is classified as an integer preventing any characters or any irrational numbers to be used when defining any pins regarding the thruster code 
- The values 0 and 15 for the maximum pin values were chosen as the PCA9685 board (the PWM driver) has 16 channels that are represented by 0-15

