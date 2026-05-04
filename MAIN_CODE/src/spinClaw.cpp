#include "Thruster.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "PCA9685.h"

// assign to claw controller right trigger

namespace {

	constexpr int FIXED_PIN = 12;      // always use pin 12 for this motor
	
	constexpr int kMinPin = 0;
	constexpr int kMaxPin = 15;

	inline double us_to_ms(int us) {
		return static_cast<double>(us) / 1000.0;
	}
}

bool Thruster::isCorrectPin(int p) const {
	return (p >= kMinPin && p <= kMaxPin);
}

double Thruster::clampPower(double p) const {
	return std::clamp(p, -1.0, 1.0);
}

int Thruster::clampPWM(int pwm_us) const {
	const int safe_min = std::max(min_us, rest - offset);
	const int safe_max = std::min(max_us, rest + offset);

	return std::clamp(pwm_us, safe_min, safe_max);
}

// Slew limiter (prevents sudden jumps)
int Thruster::applySlew(int target) const {
	const int max_delta = 20; // micro seconds per update (tune as needed)
	
	int delta = target - pwm;
	if (delta > max_delta) delta = max_delta;
	if (delta < -max_delta) delta = -max_delta;

	return pwm + delta;
}


Thruster::Thruster()
	: pin(FIXED_PIN),
	  rest(1500),
	  offset(200),
	  pwm(1500),
	  power(0.0),
	  min_us(1300),
	  max_us(1700) {
	if (!isCorrectPin(pin)) {
		throw std::invalid_argument("Spin motor pin must be in [0, 15]");
	}
}

void Thruster::setPWM(int pwm_us, PiPCA9685::PCA9685 &driver) {
	if (!isCorrectPin(pin)) {
		throw std::invalid_argument("Invalid spin motor pin");
	}

	int target = clampPWM(pwm_us);

	pwm = applySlew(target);

	const double p = static_cast<double>(pwm - rest) / static_cast<double>(offset);
	power = clampPower(p);

	driver.set_pwm_ms(pin, us_to_ms(pwm));
}

void Thruster::setPower(double pwr, PiPCA9685::PCA9685 &driver) {
	const double p = clampPower(pwr);

	const double target_us = static_cast<double>(rest) + p * static_cast<double>(offset);

	setPWM(static_cast<int>(target_us), driver);
}

void Thruster::stop(PiPCA9685::PCA9685 &driver) {
	setPWM(rest, driver);
}


int Thruster::getPin() const { return pin; }
int Thruster::getPWM() const { return pwm; }
double Thruster::getPower() const { return power; }
