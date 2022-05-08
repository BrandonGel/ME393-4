/*
   Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial
   by Dejan, https://howtomechatronics.com
*/
#ifndef MPU6050_h
#define MPU6050_h

#include "Arduino.h"
#include <Wire.h>


class MPU6050
{
  public:
    MPU6050(){};
    const int MPU = 0x68; // MPU6050 I2C address
    float AccX, AccY, AccZ;
    float GyroX, GyroY, GyroZ;
    float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
    float roll, pitch, yaw;
    float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
    
    void init();
    void calculate_IMU_error();
    void getAcc();
    
  private:
    int c = 0;  
    float elapsedTime, currentTime, previousTime;
};

#endif
