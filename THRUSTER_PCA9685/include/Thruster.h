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
  double power; // last commanded power [-1, 1]
                
  
  int min_us; // the absolute safety limit (low)
  int max_us; // the absolute safety limit (high)
              

  PiPCA9685::PCA9685* driver;   //the link to the PCA9685 hardware board


public:
  // Constructors
  Thruster();
  explicit Thruster(int pin, PiPCA9685::PCA9685* driver_ptr);
  Thruster(int pin, PiPCA9685::PCA9685* driver_ptr, int rest, int offset);

  // Sets
  void setPWM(int pwm_us, PiPCA9685::PCA9685 &external_driver);
  void setPower(double power_val, PiPCA9685::PCA9685 &external_driver);
  void stop(PiPCA9685::PCA9685 &external_driver);

  void setRest(int rest_us);
  void setOffset(int offset_us);
  void setLimits(int min_us, int max_us);

  // Gets
  // testing if this works
  int getPin() const;
  int getRest() const;
  int getOffset() const;
  int getPWM() const;
  double getPower() const;
};
