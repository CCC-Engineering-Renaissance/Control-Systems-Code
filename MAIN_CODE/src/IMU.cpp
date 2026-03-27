#include "IMU.h"
             //sensor(0x68) is made possible by a constructor on line 110 in MPU6050.h  
IMU::IMU() : sensor(0x68) {   // Initialize sensor class inside IMU default constructor
  
  ax = 0.0f;
  ay = 0.0f;
  az = 0.0f;

  gr = 0.0f;
  gp = 0.0f;
  gy = 0.0f;

  roll = 0.0f;
  pitch = 0.0f;
  yaw = 0.0f;
}

void IMU::update(){
  // From MPU6050.h we can find functions that allow us to update useful data to use later 
  sensor.getAccel(&ax, &ay, &az); // line 114 MPU6050.h
  sensor.getGyro(&gr, &gp, &gy); // line 115 MPU6050.h
  sensor.getAngle(0, &roll);
  sensor.getAngle(1, &pitch);
  sensor.getAngle(2, &yaw);

}

float IMU::getAccelX() const { return ax; }
float IMU::getAccelY() const { return ay; }
float IMU::getAccelZ() const { return az; }

float IMU::getGyroX() const { return gr; }
float IMU::getGyroY() const { return gp; }
float IMU::getGyroZ() const { return gy; }

float IMU::getRoll() const { return roll; }
float IMU::getPitch() const { return pitch; }
float IMU::getYaw() const { return yaw; }
