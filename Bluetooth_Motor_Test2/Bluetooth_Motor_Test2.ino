/*
    Name: Brandon Ho
    Date: 2/27/2022
    Title: Motor Speed Open Loop
    Description: This script works to test the motor in an open loop response
                 It goes with DataTransfer.py to get the DC gain and time constant of the motor
                 Using B port on the Arduino Motor Shield at full power
                 Adapted from https://github.com/CooperControlsLab/CU-At-Home_feedback_controls/tree/master/arduino_code/libraries/PID
*/

// Change encoder 2 b/c


#include "PinChangeInterrupt.h"


// Define hardware pins for Motor Control system
#define ENC2_A 10   //Encoder pulse A (RIGHT ENCODER)
#define ENC2_B 11   //Encoder pulse B
#define ENC1_A 12   //Encoder pulse A (LEFT ENCODER)
#define ENC1_B 13   //Encoder pulse B


#define PPR 464.64 //Encoder pulses per revolution
//#define PPR_A 11*34 //Pulse A per r
#define MOTOR_RELAY 41
#define LEFT_MOTOR_DIR 37
#define LEFT_MOTOR_PWM 45
#define RIGHT_MOTOR_DIR 35
#define RIGHT_MOTOR_PWM 44
#define LONG_LENGTH 2147483647


#define RAD_TO_DEG 565.486677646
#define DEG_TO_RAD 0.01745329251
#define RPM_TO_RADS 0.104719755
#define DEGS_TO_RPM 0.166667
#define PI 3.1415926535897932384626433832795




//Interrupt Functios
void pulse1A();
void pulse1B();
void pulse2A();
void pulse2B();



long count = 0; // This variable will increase or decrease depending on the rotation of encoder
long count2 = 0; // This variable will increase or decrease depending on the rotation of encoder


double current1;
double current2;

float pos = 0;
float pos2 = 0;
float posPrev = 0;
float posPrev2 = 0;



void setup() {
  analogReference(DEFAULT);
  //Initialize the Serial Monitor
  //delay(200);
  //Serial.flush();
  //delay(200);
  Serial.begin(19200);



  pinMode(ENC1_A, INPUT_PULLUP); // internally pullup encoder input A
  pinMode(ENC1_B, INPUT_PULLUP); // internally pullup encoder input B
  pinMode(ENC2_A, INPUT_PULLUP); // internally pullup encoder input A
  pinMode(ENC2_B, INPUT_PULLUP); // internally pullup encoder input B

  pinMode(MOTOR_RELAY, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  digitalWrite(LEFT_MOTOR_PWM, LOW);
  digitalWrite(RIGHT_MOTOR_PWM, LOW);


  //Setting up interrupt
  //When rotating the encoder, the encoder pin values changes.
  //Thus, the interrupt function will activate in the hardware itself
  attachPCINT(digitalPinToPCINT(ENC1_A), pulse1A, CHANGE);
  attachPCINT(digitalPinToPCINT(ENC1_B), pulse1B, CHANGE);
  attachPCINT(digitalPinToPCINT(ENC2_A), pulse2A, CHANGE);
  attachPCINT(digitalPinToPCINT(ENC2_B), pulse2B, CHANGE);



}



void loop()
{
  
    if (millis() / 1.0e3 > 0 && millis() / 1.0e3 < 7.5)
    
    {
      int voltage = 5.0;
     digitalWrite(MOTOR_RELAY, HIGH);
      update_motor_voltage(voltage, 1);
      update_motor_voltage(voltage, 2);
      Serial.print("help");

    }
    else if ( millis() / 1.0e3 < 15)
    {
      int voltage = -5.0;
      update_motor_voltage(voltage, 1);
      update_motor_voltage(voltage, 2);
      Serial.print("help2");
    }
    else
    {
      digitalWrite(MOTOR_RELAY, LOW);
    }
    


}


//*****************************************************//
void update_motor_voltage(double voltage, int opt) {
  //Fix direction based on +/-
  if ( opt == 1)
  {
    if (voltage > 0) {
      digitalWrite(LEFT_MOTOR_DIR, HIGH);
    }

    else {
      digitalWrite(LEFT_MOTOR_DIR, LOW);
    }
    analogWrite(LEFT_MOTOR_PWM, volts_to_PWM(abs(voltage)));
  }

  else if (opt == 2)
  {
    if (voltage > 0) {
      digitalWrite(RIGHT_MOTOR_DIR, HIGH);
    }

    else {
      digitalWrite(RIGHT_MOTOR_DIR, LOW);
    }
    analogWrite(RIGHT_MOTOR_PWM, volts_to_PWM(abs(voltage)));
  }

}



//*****************************************************//
// Voltage and PWM duty cycle conversion
int volts_to_PWM(double voltage) {

  // Convert voltage to PWM duty cycle percent relative to the power supply voltage
  return constrain(round((voltage/5.0) * 255), 0, 255);
}

//Encoder Count to Radians
double count_to_deg(double count) {
  return (double(count / PPR) * 360);  // rad = (count / pulse/rev) * 360 deg/rev
}

double sensorFusion(double a, double b, double r)
{
  return r * a + (1 - r) * b;
}

//Encoder Interrupt 1A
void pulse1A()
{
  bool valA = digitalRead(ENC1_A);
  bool valB = digitalRead(ENC1_B);

  if (valA == HIGH) { //When A rises
    if (valB == LOW) {
      count++; //CW
    }
    else {
      count--; //CCW
    }
  }
  else { //When A falls
    if (valB == HIGH) {
      count++;  //CW
    }
    else {
      count--; //CCW
    }
  }


  if ((count > LONG_LENGTH - 1000) || (count < -LONG_LENGTH + 1000))
  {
    //posPrev = posPrev - count_to_deg(count) * DEG_TO_RAD;
    count = 0;
  }
}


//Encoder Interrupt 1B
void pulse1B()
{
  bool valA = digitalRead(ENC1_A);
  bool valB = digitalRead(ENC1_B);

  if (valB == HIGH) //When B rises
  {
    if (valA == HIGH) {
      count++; //CW
    }
    else {
      count--; //CCW
    }
  }
  else //When B falls
  {
    if (valA == LOW) {
      count++; //CW
    }
    else {
      count--; //CCW
    }
  }

  if ((count > LONG_LENGTH - 1000) || (count < -LONG_LENGTH + 1000))
  {
    //posPrev = posPrev - count_to_deg(count) * DEG_TO_RAD;
    count = 0;

  }
}


//Encoder Interrupt 2A
void pulse2A()
{
  bool valA = digitalRead(ENC2_A);
  bool valB = digitalRead(ENC2_B);

  if (valA == HIGH) { //When A rises
    if (valB == LOW) {
      count2++; //CW
    }
    else {
      count2--; //CCW
    }
  }
  else { //When B falls
    if (valB == HIGH) {
      count2++;  //CW
    }
    else {
      count2--; //CCW
    }
  }


  if ((count2 > LONG_LENGTH - 1000) || (count2 < -LONG_LENGTH + 1000))
  {
    //posPrev = posPrev - count_to_deg(count) * DEG_TO_RAD;
    count2 = 0;
  }
}


//Encoder Interrupt 2B
void pulse2B()
{
  bool valA = digitalRead(ENC2_A);
  bool valB = digitalRead(ENC2_B);

  if (valB == HIGH) //When B rises
  {
    if (valA == HIGH) {
      count2++; //CW
    }
    else {
      count2--; //CCW
    }
  }
  else //When B falls
  {
    if (valA == LOW) {
      count2++; //CW
    }
    else {
      count2--; //CCW
    }
  }

  if ((count2 > LONG_LENGTH - 1000) || (count2 < -LONG_LENGTH + 1000))
  {
    //posPrev = posPrev - count_to_deg(count) * DEG_TO_RAD;
    count2 = 0;

  }
}
