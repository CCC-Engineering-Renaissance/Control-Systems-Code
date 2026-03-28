#pragma once
class PID {
public:
    PID(float kp, float ki, float kd, float min_output, float max_output);

    float update(float setpoint, float measured, float dt);

    void reset();

private:
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float min_output, max_output;
};

