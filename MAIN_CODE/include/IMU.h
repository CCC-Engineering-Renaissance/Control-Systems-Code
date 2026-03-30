#pragma once

#include "MPU6050.h"
#include <cstdint>

class IMU {

private:
  
  MPU6050 sensor;

  float ax, ay, az;
  float gr, gp, gy;
  float roll, pitch, yaw;

public:

    IMU();
    explicit IMU(int8_t address); // find out the address bruh

    void update();

    float getAccelX() const;
    float getAccelY() const;
    float getAccelZ() const;

    float getGyroX() const;
    float getGyroY() const;
    float getGyroZ() const;

    float getRoll() const;
    float getPitch() const;
    float getYaw() const;

};
