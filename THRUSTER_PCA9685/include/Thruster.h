#pragma once

namespace PiPCA9685 {
class PCA9685;
}

class Thruster {
private:
  // Helper checks and clamps
  bool isCorrectPin(int p) const;
  int clampPWM(int pwm_us) const;
  double clampPower(double power) const;

private:
  int pin;
  int rest;    // neutral pulse in microseconds (usually 1500 us)
  int offset;  // maximum deviatioin from the rest value positive/negative
               // direction (usually 400 us)
  int pwm;     // the last commanded pwm in micrseconds
  float power; // last commanded power [-1, 1]

  int min_us; // the absolute safety limit (low)
  int max_us; // the absolute safety limit (high)

public:
  // Constructors
  Thruster();
  // to be continued
};
