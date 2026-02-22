#include "Claw.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "PiPCA9685/PCA9685.h"

// Mostly reusing Thruster.cpp except Claw object is configured for Servos 
//
namespace {
  constexpr int kMinPin = 0;
  constexpr int kMaxPin = 15;
  
  inline double us_to_ms(int us){
    return static_cast<double>(us) / 1000.0;
  }
}

bool Claw::isCorrectPin(int p)) const {
  return (p >= kMinPin && p <= kMaxPin);
}

double Claw::clampPosition(double pos) const {
  // Normalize position into [-1, +1]
  // Full reverse = -1 & Full forward = +1 about origin 
  
  return std:clamp(pos, -1.0, 1.0);
}

int Claw::clampPWM(int pwm_us) const {
  int clamped = std::clamp(pwm_us, mun_us, max_us);
// don't want to clamp around plus or minus rest or offset 
  return std::clamp(pwm_us_in, )
}
// Constructors
// Section 3: Constructors (initialize safe defaults)

Claw::Claw()
  : pin(-1),          // -1 means "invalid/unassigned" until constructed with a real pin
    rest(1500),       // typical servo center pulse width (us)
    offset(400),      // default command range around center (us); adjust per mechanism
    pwm(1500),        // start at center so state is safe by default
    power(0.0f),      // cached "position" in [-1,1] (header calls it power)
    min_us(1000),     // common servo low limit (start conservative if unsure)
    max_us(2000) {    // common servo high limit
  // No hardware action here; constructors should not talk to the driver.
}

Claw::Claw(int pin_in)
  : pin(pin_in),
    rest(1500),
    offset(400),
    pwm(1500),
    power(0.0f),
    min_us(1000),
    max_us(2000) {
  // Fail fast if the channel is invalid.
  if (!isCorrectPin(pin)) {
    throw std::invalid_argument("Claw pin must be in [0,15]");
  }
  pwm = clampPWM(pwm);
}

Claw::Claw(int pin_in, int rest_in)
  : pin(pin_in),
    rest(rest_in),
    offset(400),
    pwm(rest_in),     // start at the configured center
    power(0.0f),
    min_us(1000),
    max_us(2000) {
  if (!isCorrectPin(pin)) {
    throw std::invalid_argument("Claw pin must be in [0,15]");
  }
  pwm = clampPWM(pwm);
}

Claw::Claw(int pin_in, int rest_in, int offset_in)
  : pin(pin_in),
    rest(rest_in),
    offset(std::max(0, offset_in)), // prevent negative offset (nonsensical)
    pwm(rest_in),
    power(0.0f),
    min_us(1000),
    max_us(2000) {
  if (!isCorrectPin(pin)) {
    throw std::invalid_argument("Claw pin must be in [0,15]");
  }
  pwm = clampPWM(pwm);
}

// Setters


void Claw::setPWM(int pwm_us_in, PiPCA9685::PCA9685 &driver) {
  // Guard: channel must be valid before commanding hardware.
  if (!isCorrectPin(pin)) {
    throw std::invalid_argument("Claw pin invalid (must be 0..15)");
  }

  // Clamp and store the hardware-space command (microseconds).
  pwm = clampPWM(pwm_us_in);

  // Keep cached "position" consistent with pwm (useful for telemetry/debug).
  // header calls this field "power", but here it represents servo position in [-1,1].
  if (offset > 0) {
    const double pos = static_cast<double>(pwm - rest) / static_cast<double>(offset);
    power = static_cast<float>(clampPosition(pos));
  } else {
    power = 0.0f; // avoid division by zero
  }

  // Send to PCA9685:
  // set_pwm_ms expects milliseconds.
  driver.set_pwm_ms(pin, us_to_ms(pwm));
}

void Claw::setPosition(double pos_in, PiPCA9685::PCA9685 &driver) {
  // Guard: channel must be valid before commanding hardware.
  if (!isCorrectPin(pin)) {
    throw std::invalid_argument("Claw pin invalid (must be 0..15)");
  }

  // Clamp requested control-space command.
  const double pos = clampPosition(pos_in);

  // Map normalized position -> microseconds around center.
  // Example: rest=1500, offset=400
  // pos=+1.0 => 1900 us
  // pos= 0.0 => 1500 us
  // pos=-1.0 => 1100 us
  const double target_us = static_cast<double>(rest) + pos * static_cast<double>(offset);

  // Route through setPWM so clamping + cached-state updates happen in one place.
  setPWM(static_cast<int>(std::lround(target_us)), driver);
}

void Claw::center(PiPCA9685::PCA9685 &driver) {
  // "Center" means go to rest (center pulse).
  setPWM(rest, driver);
}


// Configuration setters 


void Claw::setLimits(int min_in, int max_in) {
  // Ensure min <= max even if caller passes them reversed.
  if (min_in > max_in) {
    std::swap(min_in, max_in);
  }

  min_us = min_in;
  max_us = max_in;

  // Re-clamp current pwm to new constraints.
  pwm = clampPWM(pwm);

  // Update cached position to stay consistent.
  if (offset > 0) {
    const double pos = static_cast<double>(pwm - rest) / static_cast<double>(offset);
    power = static_cast<float>(clampPosition(pos));
  } else {
    power = 0.0f;
  }
}


// Getters

int Claw::getPin() const { return pin; }
int Claw::getRest() const { return rest; }
int Claw::getOffset() const { return offset; }
int Claw::getPWM() const { return pwm; }
double Claw::getPower() const { return static_cast<double>(power); }
    
