#pragma once

namespace PiPCA9685 {

class PCA9685 {
public:
  void set_pwm_ms(int channel_in, double ms_in) {
    last_channel = channel_in;
    last_ms = ms_in;
    call_count++;
  }

  int last_channel = -1;
  double last_ms = 0.0;
  int call_count = 0;
};

}  // namespace PiPCA9685
