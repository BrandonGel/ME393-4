#include "Arduino.h"
#include "LCD.h"

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

void LCD::init(float r1, float r2,int in)
{
  R1 = r1;
  R2 = r2;
  voltMeasure= in;
  pinMode(voltMeasure,INPUT);
  
}

float LCD::displayVolt()
{
  volt = (analogRead(voltMeasure)*(R1+R2)/R1) * 5.0 / 1023;
  if (volt - prevVolt >= 1)
  {
    lcd.begin(16, 2);
    lcd.print(voltage); // Prints "Arduino" on the LCD
  }
  currTime = millis();

  
  if((currTime - prevTime) >= 50 && state == 0)
  {
    //lcd.setCursor(voltage.length(), 0);   
    lcd.print(volt);
    lcd.print("V");
    Serial.print("printing");
    prevTime = currTime;
    state = 1;
  }
  else if((currTime - prevTime) >= 50 && state == 1)
  {
    //lcd.clear();
    //lcd.print(voltage); // Prints "Arduino" on the LCD
    lcd.setCursor(voltage.length(), 0);   
    state = 0;
    prevTime = currTime;
  }
  
   

  
  prevVolt = volt;
  
  return volt;
}
