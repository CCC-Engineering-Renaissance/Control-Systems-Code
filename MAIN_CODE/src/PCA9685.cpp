#include "PCA9685.h"
#include <unistd.h>
#include <cmath>
#include "Constants.h"
#include "I2CPeripheral.h"

namespace PiPCA9685 {

PCA9685::PCA9685(const std::string &device, int address, uint8_t mux_channel) {
  i2c_dev = std::make_unique<I2CPeripheral>(device, static_cast<uint8_t>(address), mux_channel);

  // Configure output driver and wake up with AI enabled before any block writes.
  i2c_dev->WriteRegisterByte(MODE2, OUTDRV);
  i2c_dev->WriteRegisterByte(MODE1, ALLCALL | AI);
  usleep(5'000);
  auto mode1_val = i2c_dev->ReadRegisterByte(MODE1);
  mode1_val &= ~SLEEP;
  i2c_dev->WriteRegisterByte(MODE1, mode1_val);
  usleep(5'000);

  // Zero all channels now that AI is enabled for block writes.
  set_all_pwm(0, 0);
}

PCA9685::~PCA9685() {
  try { set_all_pwm(0, 0); } catch (...) {}
}

void PCA9685::set_pwm_freq(const double freq_hz) {
  frequency = freq_hz;

  auto prescaleval = 2.5e7; //    # 25MHz
  prescaleval /= 4096.0; //       # 12-bit
  prescaleval /= freq_hz;
  prescaleval -= 1.0;

  auto prescale = static_cast<int>(std::round(prescaleval));

  const auto oldmode = i2c_dev->ReadRegisterByte(MODE1);

  auto newmode = (oldmode & 0x7F) | SLEEP;

  i2c_dev->WriteRegisterByte(MODE1, newmode);
  i2c_dev->WriteRegisterByte(PRESCALE, prescale);
  i2c_dev->WriteRegisterByte(MODE1, oldmode);
  usleep(5'000);
  i2c_dev->WriteRegisterByte(MODE1, oldmode | RESTART);
}

void PCA9685::set_pwm(const int channel, const uint16_t on, const uint16_t off) {
  const uint8_t base = static_cast<uint8_t>(LED0_ON_L + 4 * channel);
  const uint8_t data[4] = {
    static_cast<uint8_t>(on  & 0xFF),
    static_cast<uint8_t>(on  >> 8),
    static_cast<uint8_t>(off & 0xFF),
    static_cast<uint8_t>(off >> 8),
  };
  i2c_dev->WriteRegisterBlock(base, data, 4);
}

void PCA9685::set_all_pwm(const uint16_t on, const uint16_t off) {
  const uint8_t data[4] = {
    static_cast<uint8_t>(on  & 0xFF),
    static_cast<uint8_t>(on  >> 8),
    static_cast<uint8_t>(off & 0xFF),
    static_cast<uint8_t>(off >> 8),
  };
  i2c_dev->WriteRegisterBlock(ALL_LED_ON_L, data, 4);
}

void PCA9685::set_pwm_ms(const int channel, const double ms) {
  auto period_ms = 1000.0 / frequency;
  auto bits_per_ms = 4096 / period_ms;
  auto bits = ms * bits_per_ms;
  set_pwm(channel, 0, bits);
}

}  // namespace PiPCA9685
