#include "Thruster.h"              // Brings in the Thruster class declaration (must match this .cpp)

#include <algorithm>               // std::clamp, std::max, std::swap (clamping + small utilities)
#include <cmath>                   // std::lround (round double -> nearest integer)
#include <stdexcept>               // std::invalid_argument (exceptions for invalid pin)

#include "PCA9685.h"     // Full definition of PiPCA9685::PCA9685 so we can call its methods


// Section 1: File-local helpers (private to this .cpp)

namespace {
  // These constants are hardware facts for PCA9685 channel indices.
  // They are file-local so they do not pollute the public API.
  constexpr int kMinPin = 0;       // First PCA9685 PWM channel
  constexpr int kMaxPin = 15;      // Last PCA9685 PWM channel (16 channels total: 0..15)

  // Unit conversion helper:
  // Thruster uses microseconds (us). Driver helper set_pwm_ms uses milliseconds (ms).
  inline double us_to_ms(int us) {
    // Cast to double first so division is floating-point (1500/1000 -> 1.5, not 1).
    return static_cast<double>(us) / 1000.0;
  }
} // namespace


// Section 2: Helper checks and clamps (Thruster private methods)

bool Thruster::isCorrectPin(int p) const {
  // Returns true only if p is within PCA9685 channel bounds.
  // Why: the driver does not bounds-check channels; invalid channel writes wrong registers.
  return (p >= kMinPin && p <= kMaxPin);
}

double Thruster::clampPower(double p) const {
  // Forces normalized power into [-1, +1].
  // Why: control code/joystick inputs can overshoot; we never want to command beyond full-scale.
  return std::clamp(p, -1.0, 1.0);
}

int Thruster::clampPWM(int pwm_us) const {
  // Two-layer clamp:
  // 1) absolute safety limits: [min_us, max_us]
  // 2) control authority around rest: [rest-offset, rest+offset]
  //
  // This ensures hardware safety AND keeps the configured "max deviation" respected.

  // Clamp to absolute safety first.
  int clamped = std::clamp(pwm_us, min_us, max_us);

  // Compute the symmetric window around neutral.
  const int sym_min = rest - offset;
  const int sym_max = rest + offset;

  // Clamp again to the window around rest.
  clamped = std::clamp(clamped, sym_min, sym_max);

  // Return final safe PWM in microseconds.
  return clamped;
}


// Section 3: Constructors (initialize safe defaults)

Thruster::Thruster()
  : pin(-1),          // -1 means "invalid/unassigned" until you construct with a real pin
    rest(1500),       // typical ESC neutral pulse width (us)
    offset(350),      // max deviation around neutral: 1500 ± 350 = [1150, 1850]
    pwm(1500),        // start at neutral so state is safe by default
    power(0.0f),      // normalized power at neutral
    min_us(1228),     // ESC low safety limit
    max_us(1772) {    // ESC high safety limit
  // No hardware action here; constructors should not talk to the driver.
}

Thruster::Thruster(int pin_in)
  : pin(pin_in),
    rest(1500),
    offset(350),
    pwm(1500),
    power(0.0f),
    min_us(1228),
    max_us(1772) {
  // Fail fast if the channel is invalid.
  if (!isCorrectPin(pin)) {
    throw std::invalid_argument("Thruster pin must be in [0,15]");
  }
}

Thruster::Thruster(int pin_in, int rest_in)
  : pin(pin_in),
    rest(rest_in),
    offset(350),
    pwm(rest_in),     // start at the configured neutral
    power(0.0f),
    min_us(1228),
    max_us(1772) {
  if (!isCorrectPin(pin)) {
    throw std::invalid_argument("Thruster pin must be in [0,15]");
  }
  // Ensure pwm respects limits after custom rest is applied.
  pwm = clampPWM(pwm);
}

Thruster::Thruster(int pin_in, int rest_in, int offset_in)
  : pin(pin_in),
    rest(rest_in),
    offset(std::max(0, offset_in)), // prevent negative offset (nonsensical)
    pwm(rest_in),
    power(0.0f),
    min_us(1228),
    max_us(1772) {
  if (!isCorrectPin(pin)) {
    throw std::invalid_argument("Thruster pin must be in [0,15]");
  }
  pwm = clampPWM(pwm);
}


// Section 4: Commands (these actually talk to hardware via the driver reference)

void Thruster::setPWM(int pwm_us, PiPCA9685::PCA9685 &driver) {
  // Guard: channel must be valid before commanding hardware.
  if (!isCorrectPin(pin)) {
    throw std::invalid_argument("Thruster pin invalid (must be 0..15)");
  }

  // Clamp and store the hardware-space command (microseconds).
  pwm = clampPWM(pwm_us);

  // Keep cached power consistent with pwm (useful for telemetry/simulation/UI).
  if (offset > 0) {
    // Convert pwm deviation back into normalized power.
    const double p = static_cast<double>(pwm - rest) / static_cast<double>(offset);
    power = static_cast<float>(clampPower(p));
  } else {
    // If offset is 0, power has no meaning (avoid division by zero).
    power = 0.0f;
  }

  // Send to PCA9685:
  // set_pwm_ms expects milliseconds; driver converts ms -> tick counts using its configured frequency.
  driver.set_pwm_ms(pin, us_to_ms(pwm));
}

void Thruster::setPower(double pwr, PiPCA9685::PCA9685 &driver) {
  // Guard: channel must be valid before commanding hardware.
  if (!isCorrectPin(pin)) {
    throw std::invalid_argument("Thruster pin invalid (must be 0..15)");
  }

  // Clamp requested control-space command.
  double p = clampPower(pwr);
  if (inverted) p = -p;

  // Map normalized power -> microseconds around neutral.
  // Example: rest=1500, offset=350
  // power=+1.0 => 1850 us
  // power= 0.0 => 1500 us
  // power=-1.0 => 1150 us
  const double target_us = static_cast<double>(rest) + p * static_cast<double>(offset);

  // Route through setPWM so all safety clamping and cached-state updates happen in one place.
  setPWM(static_cast<int>(std::lround(target_us)), driver);
}

void Thruster::stop(PiPCA9685::PCA9685 &driver) {
  // "Stop" means go to neutral (rest pulse).
  // This is safer than assuming "0 power" always maps to exactly rest if the mapping ever changes.
  setPWM(rest, driver);
}

// Section 5: Configuration setters (update internal config; do NOT touch hardware)

void Thruster::setRest(int rest_us) {
  // Update neutral pulse width.
  rest = rest_us;

  // Re-clamp current pwm to new constraints.
  pwm = clampPWM(pwm);

  // Update cached power to stay consistent.
  if (offset > 0) {
    const double p = static_cast<double>(pwm - rest) / static_cast<double>(offset);
    power = static_cast<float>(clampPower(p));
  } else {
    power = 0.0f;
  }
}

void Thruster::setOffset(int offset_us) {
  // Offset must be non-negative.
  offset = std::max(0, offset_us);

  pwm = clampPWM(pwm);

  if (offset > 0) {
    const double p = static_cast<double>(pwm - rest) / static_cast<double>(offset);
    power = static_cast<float>(clampPower(p));
  } else {
    power = 0.0f;
  }
}

void Thruster::setLimits(int min_in, int max_in) {
  // Ensure min <= max even if caller passes them reversed.
  if (min_in > max_in) {
    std::swap(min_in, max_in);
  }

  min_us = min_in;
  max_us = max_in;

  pwm = clampPWM(pwm);

  if (offset > 0) {
    const double p = static_cast<double>(pwm - rest) / static_cast<double>(offset);
    power = static_cast<float>(clampPower(p));
  } else {
    power = 0.0f;
  }
}

// Section 6: Getters (read-only accessors)

int Thruster::getPin() const { return pin; }
int Thruster::getRest() const { return rest; }
int Thruster::getOffset() const { return offset; }
int Thruster::getPWM() const { return pwm; }
double Thruster::getPower() const { return static_cast<double>(power); }
