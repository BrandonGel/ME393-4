
#include "LCD.h" // includes the LiquidCrystal Library 
LCD lcdScreen;


void setup() 
{ 
  Serial.begin(9600);
  //lcdScreen.init(10000,0,A0);
  lcdScreen.init(10000,10000,A1);
}


void loop() { 
 
 //delay(3000); // 3 seconds delay 
  Serial.println(lcdScreen.displayVolt());
  

 
}
