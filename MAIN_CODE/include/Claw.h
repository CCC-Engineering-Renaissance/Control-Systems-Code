#pragma once 

namespace PiPCA9685 {
class PCA9685;
}

class Claw {
private:
  // Helper checks and clamps
  bool isCorrectPin(int p) const;
  int clampPWM(int pwm_us) const;
  double clampPosition(double pos) const;

private:
  int pin;
  int rest;    // neutral pulse in microseconds (usually 1500 us)
  int offset;  // maximum deviation from the rest value positive/negative
               // direction (usually 400 us)
  int pwm;     // the last commanded pwm in microseconds
  float power; // last commanded power [-1, 1]

  int min_us;  // the absolute safety limit (low)
  int max_us;  // the absolute safety limit (high)

public:
  // Constructors
  Claw();
  explicit Claw(int pin);
  Claw(int pin, int rest);
  Claw(int pin, int rest, int offset);

  // Sets
  void setPWM(int pwm_us, PiPCA9685::PCA9685 &driver);
  void setPosition(double pos, PiPCA9685::PCA9685 &driver);
  void center(PiPCA9685::PCA9685 &driver);

  void setRest(int rest_us);
  void setOffset(int offset_us);
  void setLimits(int min_us, int max_us);

  // Gets
  int getPin() const;
  int getRest() const;
  int getOffset() const;
  int getPWM() const;
  double getPower() const;

};
