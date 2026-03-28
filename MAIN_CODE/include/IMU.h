#pragma once

#include "MPU6050.h"

class IMU {

private:
  
  MPU6050 sensor;

  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, yaw 

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
