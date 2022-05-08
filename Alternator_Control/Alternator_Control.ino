#include <LiquidCrystal.h>
#include <Wire.h>

const int rs = 28, en = 29, d4 = 26, d5 = 27, d6 = 24, d7 = 25, motora_pwm = 3, motora_dir = 12, motora_brake = 9;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
#define REDLITE 46 // switched from D4 to work with PCB
#define GREENLITE 45 // switched from D5 to work with PCB
#define BLUELITE 44 // switched from D4 to work with PCB

int brightness = 255;

float sensorValue = 0.00;

int analogPin = A8; //changed from A0 to fit onto PCB
int powerRelayPin = 22;
int motorRelayPin = 23;

void setup() {
  // put your setup code here, to run once:
  lcd.begin(16, 2);

  pinMode(powerRelayPin, OUTPUT);
  digitalWrite(powerRelayPin, HIGH);

  pinMode(motorRelayPin, OUTPUT);
  digitalWrite(motorRelayPin, LOW);
  
  pinMode(REDLITE, OUTPUT);
  pinMode(GREENLITE, OUTPUT);
  pinMode(BLUELITE, OUTPUT);


  pinMode(motora_dir, OUTPUT);        //Initiates Motor Channel A direction pin
  pinMode(motora_brake, OUTPUT);      //Initiates Motor Brake Channel A direction pin
  pinMode(motora_pwm, OUTPUT);

  digitalWrite(motora_dir, HIGH);     //Motor A goes in forward direction 
  digitalWrite(motora_brake, HIGH);   //Default state should be brake is engaged
  analogWrite(motora_pwm, 0);

  // sensorValue = analogRead(analogPin);
  // String temp = " V";
  // lcd.print(sensorValue);
  // lcd.setCursor(0, 1);
  
  setBacklight(100,100,100);
  Serial.begin(9600);
}

void loop() {

  //Serial.println(analogRead(analogPin));
  displayVoltage();
  delay(50);

  if(sensorValue > 6.0){
    digitalWrite(powerRelayPin, LOW);
    digitalWrite(motorRelayPin, HIGH);
    digitalWrite(motora_brake, LOW);
    analogWrite(motora_pwm, 2);
    
//    while (sensorValue > 3.0){
//      displayVoltage();
//      // digitalWrite(powerRelayPin, LOW);
//      // digitalWrite(motorRelayPin, HIGH);
//      delay(100);
//      Serial.println("In Loop");
//    }
//    delay(50);
//    
//    digitalWrite(motorRelayPin, LOW);
//    digitalWrite(powerRelayPin, HIGH);
//    digitalWrite(motora_brake, HIGH);
//    analogWrite(motora_pwm, 0);
  }
  else{
    digitalWrite(motorRelayPin, LOW);
    digitalWrite(powerRelayPin, HIGH);
    digitalWrite(motora_brake, HIGH);
    analogWrite(motora_pwm, 0);
  }
  
  
  delay(100);
}

void displayVoltage(){
  
  // use R1 = 10, R2 = 2.4, raised term as it was slightly too low on the multimeter
  sensorValue = 5.2*5.0*(analogRead(analogPin)/1023.0);
  Serial.println(sensorValue);
  lcd.setCursor(0, 0);
  lcd.print(sensorValue);
}

void setBacklight(uint8_t r, uint8_t g, uint8_t b) {
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
