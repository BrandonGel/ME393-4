/*
    Name: Brandon Ho
    Date: 2/27/2022
    Title: Motor Speed Open Loop
    Description: This script works to test the motor in an open loop response
                 It goes with DataTransfer.py to get the DC gain and time constant of the motor
                 Using B port on the Arduino Motor Shield at full power
                 Adapted from https://github.com/CooperControlsLab/CU-At-Home_feedback_controls/tree/master/arduino_code/libraries/PID
*/

int SENSOR_PIN = 0; // center pin of the potentiometer

//Motor Shield Pin Values
const int leftDirection = 12;
const int leftSpeed = 3;
const int leftBrake = 9;
const int rightDirection = 13;
const int rightSpeed = 11;
const int rightBrake = 8;


void setup() {
  //Initialize the Serial Monitor
  //delay(200);
  Serial.flush();
  //delay(200);
  Serial.begin(19200);
  //Serial.println("Program Running");


  //Motor Shield Pins
  pinMode(leftDirection, OUTPUT);
  pinMode(leftSpeed, OUTPUT);
  pinMode(leftBrake, OUTPUT);
  pinMode(rightDirection, OUTPUT);
  pinMode(rightSpeed, OUTPUT);
  pinMode(rightBrake, OUTPUT);



   //Initalize the Motor Speed & Direction
  digitalWrite(leftBrake, LOW);   //brake A
  digitalWrite(rightBrake, LOW);   //brake B
  analogWrite(leftSpeed, 0);   //speed A
  analogWrite(rightSpeed, 0);    //speed B
  digitalWrite(leftDirection, LOW); //forward A
  digitalWrite(rightDirection, LOW); //forward B

}


void loop()
{
  int sensorValue = analogRead(SENSOR_PIN);
 Serial.println(sensorValue);
 // sensor value is in the range 0 to 1023
 // the lower half of it we use for reverse rotation; the upper half for forward

 if (sensorValue < 512)
  {
    digitalWrite(rightDirection, LOW); // CCW
    analogWrite(rightSpeed,(511-sensorValue)/2);
  }
  else
  {
    digitalWrite(rightDirection, HIGH); // CW
    analogWrite(rightSpeed,(sensorValue-512)/2);
  }
 
}
