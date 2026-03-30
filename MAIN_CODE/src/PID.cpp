#include "PID.h"
#include <algorithm>

PID::PID(float kp, float ki, float kd, float min_out, float max_out)
    : Kp(kp), Ki(ki), Kd(kd),
      integral(0.0f), prev_error(0.0f),
      min_output(min_out), max_output(max_out) {}

float PID::update(float setpoint, float measured, float dt) {
    // Protect against divide-by-zero or bad timing
    if (dt <= 0.0f) {
        return 0.0f;
    }

    float error = setpoint - measured;

    // Integral term
    integral += error * dt;

    // Anti-windup: keep integral from growing too large
    const float integral_limit = 100.0f;
    integral = std::clamp(integral, -integral_limit, integral_limit);

    // Derivative term
    float derivative = (error - prev_error) / dt;

    // Optional derivative clamp to reduce spikes/noise
    const float derivative_limit = 50.0f;
    derivative = std::clamp(derivative, -derivative_limit, derivative_limit);

    // PID output
    float output = Kp * error + Ki * integral + Kd * derivative;

    // Save error for next update
    prev_error = error;

    // Clamp final output
    return std::clamp(output, min_output, max_output);
}

void PID::reset() {
    integral = 0.0f;
    prev_error = 0.0f;
}
