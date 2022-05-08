#include "Arduino.h"
#include "LCD.h"

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

void LCD::ref(int refer)
{
  refMeasure = refer;
  pinMode(refMeasure,INPUT);
}

void LCD::init(float r1, float r2,int in)
{
  R1 = r1;
  R2 = r2;
  rectMeasure= in;
  pinMode(rectMeasure,INPUT);
  lcd.setCursor(0, 0);   
  lcd.print(voltageR); // Prints "Arduino" on the LCD  
}

void LCD::init2(float r3, float r4,int in2)
{
  R3 = r3;
  R4 = r4;
  capMeasure= in2;
  pinMode(capMeasure,INPUT);
  lcd.setCursor(0, 1);   
  lcd.print(voltageC); // Prints "Arduino" on the LCD
  
}

float LCD::displayVolt()
{
//  int ref =  analogRead(refMeasure);
  voltR = ((analogRead(rectMeasure)-analogRead(refMeasure))*(R1+R2)/R1) * 5.0 / 1023.0;
  voltC = ((analogRead(capMeasure)-analogRead(refMeasure))*(R3+R4)/R3) * 5.0 / 1023.0;
//Serial.println(ref);
//Serial.println(voltR);
//    Serial.println(voltC);
  if(voltR < 0){
    voltR = 0;
  }
  if(voltC < 0){
    voltC = 0;
  }
  
  if (voltR - prevVoltR >= 1 or voltC - prevVoltC >= 1)
  {
    lcd.begin(16, 2);
    lcd.setCursor(0, 0);   
    lcd.print(voltageR); // Prints "Arduino" on the LCD
    lcd.setCursor(0, 1);   
    lcd.print(voltageC); // Prints "Arduino" on the LCD  
  }
  currTime = millis();

  
  if((currTime - prevTime) >= 50 && state == 0)
  {
    lcd.setCursor(voltageR.length(), 0);   
    lcd.print(voltR);
    lcd.print("V");
    
    lcd.setCursor(voltageC.length(), 1);   
    lcd.print(voltC);
    lcd.print("V");
//    Serial.println("printing");
//
//    Serial.println(voltR);
//    Serial.println(voltC);
    prevTime = currTime;
    state = 1;
  }
  
  else if((currTime - prevTime) >= 50 && state == 1)
  {
    //lcd.clear();
    //lcd.print(voltage); // Prints "Arduino" on the LCD
    //lcd.setCursor(voltage.length(), 0);   
    state = 0;
    prevTime = currTime;
  }
  
  prevVoltR = voltR;
  prevVoltC = voltC;
  
  return voltC;
}

float LCD::displayVolt2()
{
  voltC = (analogRead(capMeasure)*(R3+R4)/R3) * 5.0 / 1023;
  
  if (voltR - prevVoltR >= 1 )
  {
    lcd.begin(16, 2);
    lcd.setCursor(0, 1);   
    lcd.print(voltageC); // Prints "Arduino" on the LCD  
  }
  currTime = millis();

  
  if((currTime - prevTime) >= 50 && state == 0)
{
    lcd.setCursor(voltageC.length(), 1);   
    lcd.print(voltC);
    lcd.print("V");
    Serial.println("printing");
    Serial.println(analogRead(capMeasure) * 5.0 / 1023);
//    Serial.println(voltC);
    prevTime = currTime;
    state = 1;
  }
  
  else if((currTime - prevTime) >= 50 && state == 1)
  {
    //lcd.clear();
    //lcd.print(voltage); // Prints "Arduino" on the LCD
    //lcd.setCursor(voltage.length(), 0);   
    state = 0;
    prevTime = currTime;
  }
  
  
  prevVoltC = voltC;
  
  return voltC;
}
