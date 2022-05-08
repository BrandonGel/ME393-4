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
    LCD(){lcd.begin(16, 2);};

    void ref(int refer);
    void init(float r1,float r2, int in);
    void init2(float r3,float r4, int in2);
    float displayVolt();
    float displayVolt2();
    
    
    
  private:
   
    String voltageR = "Rec_Volt: ";
    String voltageC = "Cap_Volt: "; 
    int refMeasure,rectMeasure,capMeasure;
    
    float voltR,prevVoltR = 0;
    float voltC,prevVoltC = 0;
    float R1,R2,R3,R4;
    bool state = 0;
    float currTime = 0;
    float prevTime = 0;

};

#endif
