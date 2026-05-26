#pragma once
#include <cstdint>
#include <string>

// Driver for the Blue Robotics Bar02 / Bar30 depth + pressure sensor.
// Both use the Measurement Specialties MS5837 chip over I2C at address 0x76.
// The only difference between the two is the model constant (which selects
// the pressure conversion math) and the depth range.
//
// NOTE: a full read() takes up to ~40ms because the chip needs conversion
// delays. Do NOT call read() inside the main control loop. Run it from a
// dedicated thread (see DepthSensor.cpp usage notes) at a few Hz.

class DepthSensor {
public:
  enum Model : uint8_t {
    MS5837_30BA = 0,   // Bar30 — up to 30 bar (~300 m)
    MS5837_02BA = 1,   // Bar02 — up to  2 bar (~10 m)
  };

  // i2cDevice e.g. "/dev/i2c-1"  (same bus as the PCA9685)
  // address is fixed at 0x76 for the MS5837
  explicit DepthSensor(const std::string& i2cDevice = "/dev/i2c-1",
                       uint8_t address = 0x76,
                       Model model = MS5837_30BA);
  ~DepthSensor();

  // Opens the bus, resets the chip, and reads its factory calibration PROM.
  // Returns false if the device is missing or the calibration CRC is bad.
  bool init();

  // Fluid density in kg/m^3. 1029 = seawater (default), 997 = freshwater.
  void setFluidDensity(float density) { fluidDensity_ = density; }

  // Surface pressure in mbar used as the zero reference for depth.
  // Call this once at the surface before diving to zero the depth.
  void setSurfacePressure(float mbar) { surfaceMbar_ = mbar; }
  float getSurfacePressure() const { return surfaceMbar_; }

  // Triggers a pressure + temperature conversion and reads the result.
  // Blocks ~40ms. Returns false on any I2C error. On success the getters
  // below return fresh values.
  bool read();

  float getPressureMbar()  const { return pressureMbar_;  }
  float getTemperatureC()  const { return temperatureC_;  }
  // Depth in meters relative to the surface pressure reference.
  float getDepthMeters()   const { return depthMeters_;   }

private:
  bool    writeCommand(uint8_t cmd);
  bool    readBytes(uint8_t* dst, int n);
  uint32_t readAdc(uint8_t convertCmd);   // triggers conversion, returns 24-bit ADC
  uint8_t crc4(uint16_t* prom);           // validate factory calibration
  void    calculate(uint32_t D1, uint32_t D2);  // chip's compensation math

  std::string i2cDevice_;
  uint8_t     address_;
  Model       model_;
  int         fd_ = -1;

  uint16_t C_[8] = {0};   // factory calibration coefficients from PROM

  float fluidDensity_ = 1029.0f;   // seawater
  float surfaceMbar_  = 1013.25f;  // standard atmosphere until zeroed

  float pressureMbar_  = 0.0f;
  float temperatureC_  = 0.0f;
  float depthMeters_   = 0.0f;
};
