#pragma once
class PID {
public:
    PID(float kp, float ki, float kd, float min_output, float max_output);

    float update(float setpoint, float measured, float dt);

    void reset();

    float getKp() const { return Kp; }
    float getKi() const { return Ki; }
    float getKd() const { return Kd; }

private:
    float Kp, Ki, Kd;
    float integral = 0.0f;
    float prev_error = 0.0f;
    float min_output, max_output;
};

