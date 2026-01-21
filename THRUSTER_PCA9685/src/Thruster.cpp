#include "Thruster.h"
#include "PiPCA9685/PCA9685.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
using std::clamp;    //Just because I can... (see result on line 22)

namespace {
  constexpr int MIN_PIN = 0;
  constexpr int MAX_PIN = 15;

  constexpr int PERIOD_US = 10000; //100Hz frequency this will help lower latency
  constexpr double PCA_BITS = 4096.0; //12 bit resolution
  constexpr double MAX_SAFE_POWER = 0.8; //caps maximum power to 80%, this may even not be needed but can alwasy change to 1.0 if it works fine
                                         
  //  Driver Code is written as if PWM values are in ms so we need to convert the values as we write PWM
  inline double us_to_ms(int us) { 
    return static_cast<double>(us) / 1000.0; 
    }
} 
//constructors 
Thruster::Thruster()
  : pin(0), rest(1500), offset(400), pwm(1500), power(0.0), min_us(1100), max_us(1900), driver(nullptr) {}

Thruster::Thruster(int p, PiPCA9685::PCA9685* driver_ptr)
  : pin(p), rest(1500), offset(400), pwm(1500), power(0.0), min_us(1100), max_us(1900), driver(driver_ptr) {}

Thruster::Thruster(int p, PiPCA9685::PCA9685* driver_ptr, int r, int o)
  : pin(p), rest(r), offset(o), pwm(r), power(0.0), min_us(1100), max_us(1900), driver(driver_ptr) {}

    // Helper checks and clamps
bool Thruster::isCorrectPin(int p) const{
  return (p >= MIN_PIN && p <= MAX_PIN);
}

double Thruster::clampPower(double p) const {
  //80% power cap here 
  return clamp(p, -MAX_SAFE_POWER, MAX_SAFE_POWER);
}

int Thruster::clampPWM(int pwm_us) const {
  //clamp within absolute safety limits
  return clamp(pwm_us, min_us, max_us);
}

// the command functions
void Thruster::setPower(double power_val, PiPCA9685::PCA9685 &external_driver) {
  //apply 80% cap
  double capped = clampPower(power_val);
  this->power = capped;

  //map to microseconds (rest + power * offset)
  int target_us = rest + static_cast<int>(std::round(capped * offset));
  this->pwm = clampPWM(target_us);

  //convert to 100Hz by this formula: (target time / 10000us) * 4096 bits
  double pulseRatio = static_cast<double>(this->pwm) / PERIOD_US;
  uint16_t offTick = static_cast<uint16_t>(std::round(pulseRatio * PCA_BITS));
  
  if (isCorrectPin(pin)) {
    external_driver.set_pwm(pin, 0, offTick);
  }

}



void Thruster::setPWM(int pwm_us, PiPCA9685::PCA9685 &external_driver) {
  this->pwm = clampPWM(pwm_us);
  
  double pulseRatio = static_cast<double>(this->pwm) / PERIOD_US;
  uint16_t offTick = static_cast<uint16_t>(std::round(pulseRatio * PCA_BITS));
  
  if (isCorrectPin(pin)) {
    external_driver.set_pwm(pin, 0, offTick);
  }
}

// stop() acts as a safety kill-switch. 
// We put it here to ensure the thruster returns to the 1500us 'Neutral' 
// state immediately, effectively cutting motor thrust to zero.
void Thruster::stop(PiPCA9685::PCA9685 &external_driver) {
  setPower(0.0, external_driver);
}

//setters and getters
void Thruster::setRest(int r) { rest = r; }
void Thruster::setOffset(int o) { offset = o; }
void Thruster::setLimits(int min, int max) { min_us = min; max_us = max; }

int Thruster::getPin() const { return pin; }
int Thruster::getRest() const { return rest; }
int Thruster::getOffset() const { return offset; }
int Thruster::getPWM() const { return pwm; }
double Thruster::getPower() const { return power; }


//works with stored pointer
void Thruster::setPower(double power_val) {
    if (this->driver != nullptr) {
        // This calls the version above and passes the stored driver!
        setPower(power_val, *(this->driver));
    }
}

// Internal version: Allows for a quick emergency stop without 
// needing to find the driver reference in the main loop
void Thruster::stop() {
    if (this->driver != nullptr) {
        stop(*(this->driver));
    }
}
