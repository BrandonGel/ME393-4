/*
   Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial
   by Dejan, https://howtomechatronics.com
*/
#ifndef LCD_h
#define LCD_h

#include "Arduino.h"

#include <LiquidCrystal.h> // includes the LiquidCrystal Library 

const int RS = 22, EN = 23, D4 = 24, D5 = 25, D6 = 26, D7 = 27;
extern LiquidCrystal lcd;

class LCD
{
  public:
    LCD(){};
    
    void init(float R1,float R2, int in);
    float displayVolt();
    
    
  private:
   
    String voltage = "Voltage: "; 
    int voltMeasure;
    float volt,prevVolt;
    float R1,R2;
    bool state = 0;
    float currTime = 0;
    float prevTime = 0;

};

#endif
