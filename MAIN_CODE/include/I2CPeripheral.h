#ifndef PIPCA9685_I2CPERIPHERAL_H
#define PIPCA9685_I2CPERIPHERAL_H

#include <cstdint>
#include <string>

namespace PiPCA9685 {

class I2CPeripheral {
public:
  // mux_channel: TCA9548A channel mask (e.g. TCA9548A::kCh3). Pass 0 if no mux.
  I2CPeripheral(const std::string& device, const uint8_t address, const uint8_t mux_channel = 0);
  ~I2CPeripheral();

  void WriteRegisterByte(const uint8_t register_address, const uint8_t value);

  void WriteRegisterBlock(uint8_t register_address, const uint8_t* data, size_t len);

  uint8_t ReadRegisterByte(const uint8_t register_address);

private:
  int bus_fd;
  uint8_t device_address_;
  uint8_t mux_channel_;

  void OpenBus(const std::string& device);
  void ConnectToPeripheral(const uint8_t address);

};

}  // namespace PiPCA9685

#endif  // PIPCA9685_I2CPERIPHERAL_H