#include "PID.h"
#include <algorithm>

PID::PID(float kp, float ki, float kd, float min_out, float max_out)
    : Kp(kp), Ki(ki), Kd(kd),
      integral(0), prev_error(0),
      min_output(min_out), max_output(max_out) {}

float PID::update(float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    integral += error * dt;
    float derivative = (error - prev_error)/dt;

    float output = Kp*error + Ki*integral + Kd*derivative;
    prev_error = error;

    return std::clamp(output, min_output, max_output);
}

void PID::reset() {
    integral = 0;
    prev_error = 0;
}

