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


float LCD::displayVolt2()
{
//  Serial.print("Measure: ");Serial.println(analogRead(capMeasure));
  
  voltC = (float(analogRead(capMeasure))*(R3+R4)/R4) * 5.0 / 1023.0;
//  Serial.println(voltC);
  
  if(voltC < 0){
    voltC = 0;
  }
  
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


void LCD::setBacklight(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {

  pinMode(REDLITE, OUTPUT);
  pinMode(GREENLITE, OUTPUT);
  pinMode(BLUELITE, OUTPUT);
  
  
  // normalize the red LED - its brighter than the rest!
  r = map(r, 0, 255, 0, 100);
  g = map(g, 0, 255, 0, 150);
 
  r = map(r, 0, 255, 0, brightness);
  g = map(g, 0, 255, 0, brightness);
  b = map(b, 0, 255, 0, brightness);
 
  // common anode so invert!
  r = map(r, 0, 255, 255, 0);
  g = map(g, 0, 255, 255, 0);
  b = map(b, 0, 255, 255, 0);

  analogWrite(REDLITE, r);
  analogWrite(GREENLITE, g);
  analogWrite(BLUELITE, b);
}
