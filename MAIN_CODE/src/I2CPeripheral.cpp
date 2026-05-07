#include "I2CPeripheral.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <cstring>
extern "C" {
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}
#include <system_error>

namespace PiPCA9685 {

I2CPeripheral::I2CPeripheral(const std::string& device, const uint8_t address) {
  OpenBus(device);
  ConnectToPeripheral(address);
}

I2CPeripheral::~I2CPeripheral() {
  close(bus_fd);
}

void I2CPeripheral::WriteRegisterByte(const uint8_t register_address, const uint8_t value) {
  i2c_smbus_data data;
  data.byte = value;
  for (int attempt = 0; attempt < 3; ++attempt) {
    const auto err = i2c_smbus_access(bus_fd, I2C_SMBUS_WRITE, register_address, I2C_SMBUS_BYTE_DATA, &data);
    if (!err) return;
    const int e = errno;
    if (e != EIO && e != ENXIO && e != 121) {  // 121 = EREMOTEIO on ARM Linux
      const auto msg = "Could not write value (" + std::to_string(value) + ") to register " + std::to_string(register_address);
      throw std::system_error(e, std::system_category(), msg);
    }
    if (attempt < 2) usleep(500);
  }
  const auto msg = "Could not write value (" + std::to_string(value) + ") to register " + std::to_string(register_address);
  throw std::system_error(errno, std::system_category(), msg);
}

void I2CPeripheral::WriteRegisterBlock(uint8_t register_address, const uint8_t* data, size_t len) {
  // Single write() sends register address + all data bytes in one I2C transaction.
  // Requires AI (auto-increment) bit set in MODE1 so the PCA9685 increments the
  // internal register pointer for each successive byte.
  uint8_t buf[32];
  buf[0] = register_address;
  std::memcpy(buf + 1, data, len);
  for (int attempt = 0; attempt < 3; ++attempt) {
    if (::write(bus_fd, buf, len + 1) == static_cast<ssize_t>(len + 1)) return;
    const int e = errno;
    if (e != EIO && e != ENXIO && e != 121) {  // 121 = EREMOTEIO on ARM Linux
      throw std::system_error(e, std::system_category(), "I2C block write failed");
    }
    if (attempt < 2) usleep(500);
  }
  throw std::system_error(errno, std::system_category(), "I2C block write failed");
}

uint8_t I2CPeripheral::ReadRegisterByte(const uint8_t register_address) {
  i2c_smbus_data data;
  for (int attempt = 0; attempt < 3; ++attempt) {
    const auto err = i2c_smbus_access(bus_fd, I2C_SMBUS_READ, register_address, I2C_SMBUS_BYTE_DATA, &data);
    if (!err) return data.byte & 0xFF;
    const int e = errno;
    if (e != EIO && e != ENXIO && e != 121) {  // 121 = EREMOTEIO on ARM Linux
      const auto msg = "Could not read value at register " + std::to_string(register_address);
      throw std::system_error(-err, std::system_category(), msg);
    }
    if (attempt < 2) usleep(500);
  }
  const auto msg = "Could not read value at register " + std::to_string(register_address);
  throw std::system_error(EIO, std::system_category(), msg);
}

void I2CPeripheral::OpenBus(const std::string& device) {
  bus_fd = open(device.c_str(), O_RDWR);
  if(bus_fd < 0) {
    throw std::system_error(errno, std::system_category(), "Could not open i2c bus.");
  }
}

void I2CPeripheral::ConnectToPeripheral(const uint8_t address) {
  if(ioctl(bus_fd, I2C_SLAVE, address) < 0) {
    throw std::system_error(errno, std::system_category(), "Could not set peripheral address.");
  }
}

}  // namespace PiPCA9685
