
#include <LiquidCrystal.h> // includes the LiquidCrystal Library 
LiquidCrystal lcd(22, 23, 24, 25, 26, 27); // Creates an LCD object. Parameters: (rs, enable, d4, d5, d6, d7) 

String voltage = "Voltage: ";
int arr[5] = {0,1,2,3,4};


void setup() { 
 lcd.begin(16,2); // Initializes the interface to the LCD screen, and specifies the dimensions (width and height) of the display } 
   lcd.print(voltage); // Prints "Arduino" on the LCD 
}


void loop() { 
 
 //delay(3000); // 3 seconds delay 


 for(float i = 0; i < 5; i++)
 {
   //lcd.setCursor(voltage.length(),0); // Sets the location at which subsequent text written to the LCD will be displayed 
    lcd.setCursor(voltage.length(), 0);  
   lcd.print(String(i)); 
   lcd.print("V"); 
   delay(500); 
   

  
 }
 
}
