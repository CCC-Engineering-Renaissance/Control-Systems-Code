#include "DepthSensor.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <chrono>
#include <cstring>
#include <thread>

// ── MS5837 command set ──────────────────────────────────────────────────────
namespace {
  constexpr uint8_t CMD_RESET     = 0x1E;
  constexpr uint8_t CMD_ADC_READ  = 0x00;
  constexpr uint8_t CMD_PROM_READ = 0xA0;   // base address; +2 per coefficient
  // OSR=8192 (highest resolution). Pressure D1 = 0x48, Temperature D2 = 0x58.
  constexpr uint8_t CMD_CONVERT_D1 = 0x48;
  constexpr uint8_t CMD_CONVERT_D2 = 0x58;

  void sleepMs(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  }
}

DepthSensor::DepthSensor(const std::string& i2cDevice, uint8_t address, Model model)
    : i2cDevice_(i2cDevice), address_(address), model_(model) {}

DepthSensor::~DepthSensor() {
  if (fd_ >= 0) ::close(fd_);
}

bool DepthSensor::writeCommand(uint8_t cmd) {
  return ::write(fd_, &cmd, 1) == 1;
}

bool DepthSensor::readBytes(uint8_t* dst, int n) {
  return ::read(fd_, dst, n) == n;
}

bool DepthSensor::init() {
  fd_ = ::open(i2cDevice_.c_str(), O_RDWR);
  if (fd_ < 0) return false;

  if (::ioctl(fd_, I2C_SLAVE, address_) < 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // Reset the chip and let it reload its PROM
  if (!writeCommand(CMD_RESET)) return false;
  sleepMs(10);

  // Read the 7 PROM words (C0 = factory/CRC, C1..C6 = calibration)
  for (uint8_t i = 0; i < 7; ++i) {
    if (!writeCommand(CMD_PROM_READ + i * 2)) return false;
    uint8_t buf[2] = {0, 0};
    if (!readBytes(buf, 2)) return false;
    C_[i] = (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
  }

  // Validate the calibration with the chip's CRC4
  uint8_t crcRead = (C_[0] >> 12) & 0x0F;
  uint8_t crcCalc = crc4(C_);
  return crcRead == crcCalc;
}

uint32_t DepthSensor::readAdc(uint8_t convertCmd) {
  writeCommand(convertCmd);
  // OSR=8192 needs up to ~20ms to convert; 20ms is the datasheet max.
  sleepMs(20);

  if (!writeCommand(CMD_ADC_READ)) return 0;
  uint8_t buf[3] = {0, 0, 0};
  if (!readBytes(buf, 3)) return 0;

  return (static_cast<uint32_t>(buf[0]) << 16) |
         (static_cast<uint32_t>(buf[1]) << 8)  |
          static_cast<uint32_t>(buf[2]);
}

bool DepthSensor::read() {
  if (fd_ < 0) return false;

  uint32_t D1 = readAdc(CMD_CONVERT_D1);  // raw pressure
  uint32_t D2 = readAdc(CMD_CONVERT_D2);  // raw temperature
  if (D1 == 0 || D2 == 0) return false;

  calculate(D1, D2);
  return true;
}

// First-order + second-order temperature compensation, straight from the
// MS5837 datasheet. The 02BA and 30BA differ only in a few scaling shifts.
void DepthSensor::calculate(uint32_t D1, uint32_t D2) {
  int32_t  dT   = static_cast<int32_t>(D2) - static_cast<int32_t>(C_[5]) * 256;
  int64_t  SENS, OFF;
  int32_t  TEMP;

  if (model_ == MS5837_02BA) {
    int64_t SENS_ = static_cast<int64_t>(C_[1]) * 65536 +
                    (static_cast<int64_t>(C_[3]) * dT) / 128;
    int64_t OFF_  = static_cast<int64_t>(C_[2]) * 131072 +
                    (static_cast<int64_t>(C_[4]) * dT) / 64;
    SENS = SENS_;
    OFF  = OFF_;
  } else {  // MS5837_30BA
    int64_t SENS_ = static_cast<int64_t>(C_[1]) * 32768 +
                    (static_cast<int64_t>(C_[3]) * dT) / 256;
    int64_t OFF_  = static_cast<int64_t>(C_[2]) * 65536 +
                    (static_cast<int64_t>(C_[4]) * dT) / 128;
    SENS = SENS_;
    OFF  = OFF_;
  }

  TEMP = 2000 + (static_cast<int64_t>(dT) * C_[6]) / 8388608;

  // ── Second-order temperature compensation ──
  int64_t Ti = 0, OFFi = 0, SENSi = 0;
  if (model_ == MS5837_02BA) {
    if (TEMP < 2000) {  // low temperature
      Ti    = (11 * static_cast<int64_t>(dT) * dT) / 34359738368LL;
      OFFi  = (31 * static_cast<int64_t>(TEMP - 2000) * (TEMP - 2000)) / 8;
      SENSi = (63 * static_cast<int64_t>(TEMP - 2000) * (TEMP - 2000)) / 32;
    }
  } else {  // MS5837_30BA
    if (TEMP < 2000) {  // low temperature
      Ti    = (3 * static_cast<int64_t>(dT) * dT) / 8589934592LL;
      OFFi  = (3 * static_cast<int64_t>(TEMP - 2000) * (TEMP - 2000)) / 2;
      SENSi = (5 * static_cast<int64_t>(TEMP - 2000) * (TEMP - 2000)) / 8;
      if (TEMP < -1500) {  // very low temperature
        OFFi  += 7 * static_cast<int64_t>(TEMP + 1500) * (TEMP + 1500);
        SENSi += 4 * static_cast<int64_t>(TEMP + 1500) * (TEMP + 1500);
      }
    }
  }

  OFF  -= OFFi;
  SENS -= SENSi;

  int64_t P;
  if (model_ == MS5837_02BA) {
    P = (((static_cast<int64_t>(D1) * SENS) / 2097152) - OFF) / 32768;
    pressureMbar_ = P / 100.0f;   // 02BA: result is in 0.01 mbar
  } else {
    P = (((static_cast<int64_t>(D1) * SENS) / 2097152) - OFF) / 8192;
    pressureMbar_ = P / 10.0f;    // 30BA: result is in 0.1 mbar
  }

  temperatureC_ = (TEMP - Ti) / 100.0f;

  // Depth from pressure:  depth = (P_abs - P_surface) * 100 / (rho * g)
  // pressure in Pa = mbar * 100;  g = 9.80665
  const float g = 9.80665f;
  depthMeters_ = (pressureMbar_ - surfaceMbar_) * 100.0f / (fluidDensity_ * g);
}

// CRC4 calculation from the MS5837 datasheet — validates the PROM contents.
uint8_t DepthSensor::crc4(uint16_t* prom) {
  uint16_t n_rem = 0;
  uint16_t orig0 = prom[0];

  prom[0] = prom[0] & 0x0FFF;   // strip the CRC nibble for the calculation
  uint16_t saved7 = prom[7];
  prom[7] = 0;                  // coefficient 7 is not used in the CRC

  for (int i = 0; i < 16; ++i) {
    if (i % 2 == 1) n_rem ^= (prom[i >> 1]) & 0x00FF;
    else            n_rem ^= (prom[i >> 1] >> 8);

    for (int n_bit = 8; n_bit > 0; --n_bit) {
      if (n_rem & 0x8000) n_rem = (n_rem << 1) ^ 0x3000;
      else                n_rem = (n_rem << 1);
    }
  }

  n_rem = (n_rem >> 12) & 0x000F;
  prom[0] = orig0;              // restore the words we touched
  prom[7] = saved7;
  return static_cast<uint8_t>(n_rem);
}
