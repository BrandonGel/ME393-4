/*
    Name: Brandon Ho
    Date: 2/27/2022
    Title: Motor Speed Open Loop
    Description: This script works to test the motor in an open loop response
                 It goes with DataTransfer.py to get the DC gain and time constant of the motor
                 Using B port on the Arduino Motor Shield at full power
                 Adapted from https://github.com/CooperControlsLab/CU-At-Home_feedback_controls/tree/master/arduino_code/libraries/PID
*/


#include "MPU6050.h"
#include "PID.h"
#include "BTU7960.h"

// Define hardware pins for Motor Control system
#define ENC1_A 18   //Encoder pulse A
#define ENC1_B 19   //Encoder pulse B
#define ENC2_A 20   //Encoder pulse A
#define ENC2_B 21   //Encoder pulse B

#define PPR 464.64 //Encoder pulses per revolution
//#define PPR_A 11*34 //Pulse A per r



#define MOVING_AVERAGE_SIZE 50 // Size of moving average

#define RAD_TO_DEG 360/(2*3.14159)
#define DEG_TO_RAD 2*3.14159/360
#define RPM_TO_RADS 0.104719755
#define DEGS_TO_RPM 0.166667
#define PI 3.1415926535897932384626433832795

#define LONG_LENGTH 2147483647


MPU6050 IMU;

// dirty derivative inputs for D control
double Ts = .01; //sample period in milliseconds
double sigma = 0.05; //sample period in milliseconds

// dirty derivative inputs for velocity
double sigmaA = 0.1; //sample period in milliseconds
double betaA = (2.0 * sigmaA - Ts) / (2.0 * sigmaA + Ts); //sample period in milliseconds;

PID CONTROL1(Ts, sigma, 1, 2, 0.001);
PID CONTROL2(Ts, sigma, 1, 2, 0.001);
BTU7960 MotorControl1(5, 6);



//Global VariablesUnsat
double SUPPLY_VOLTAGE  = 12; //12V power supply


const int power = 4;
const int volt_measure = A0;

//Interrupt Functios
void pulse1A();
void pulse1B();
void pulse2A();
void pulse2B();


float setpoint;
long count = 0; // This variable will increase or decrease depending on the rotation of encoder
long count2 = 0; // This variable will increase or decrease depending on the rotation of encoder


double timestamp = 0;


double voltage1;
double voltage2;
float pos = 0;
float pos2 = 0;
float posPrev = 0;
float posPrev2 = 0;
long prevT = 0;
float deltaT = 0;

long refT;

void setup() {
  //Initialize the Serial Monitor
  //delay(200);
  Serial.flush();
  //delay(200);
  Serial.begin(19200);
  //Serial.println("Program Running");

  pinMode(power, OUTPUT);
  digitalWrite(power, HIGH);
  //Motor Shield Pins


  pinMode(volt_measure, INPUT);


  pinMode(ENC1_A, INPUT_PULLUP); // internally pullup encoder input A
  pinMode(ENC1_B, INPUT_PULLUP); // internally pullup encoder input B
  pinMode(ENC2_A, INPUT_PULLUP); // internally pullup encoder input A
  pinMode(ENC2_B, INPUT_PULLUP); // internally pullup encoder input B



  //Setting up interrupt
  //When rotating the encoder, the encoder pin values changes.
  //Thus, the interrupt function will activate in the hardware itself
  attachInterrupt(digitalPinToInterrupt(ENC1_A), pulse1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), pulse1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), pulse2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), pulse2B, CHANGE);

  //Initalize the Motor Speed & Direction

  //IMU.init();

 
}

float vel = 0;
float vel2 = 0;

int state = 1;

void loop()
{
  //IMU.getAcc();
  SUPPLY_VOLTAGE = 6;//analogRead(volt_measure)*10.0/1024;
  CONTROL1.readLimits(-SUPPLY_VOLTAGE, SUPPLY_VOLTAGE);
  CONTROL2.readLimits(-SUPPLY_VOLTAGE, SUPPLY_VOLTAGE);

  pos = count_to_deg(count) * DEG_TO_RAD; // in [rad]
  pos2 = count_to_deg(count2) * DEG_TO_RAD; // in [rad]

  long currT = micros(); // in [us]
  deltaT = ((float) (currT - prevT)) / 1.0e6; // in [s]
  if(deltaT >= Ts)
  {
    vel = betaA * vel + (1 - betaA) / deltaT * (pos - posPrev); // in [rad/s]
    vel2 = betaA * vel2 + (1 - betaA) * (pos2 - posPrev2) / deltaT; // in [rad/s]
    posPrev = pos;
    posPrev2 = pos2;
    prevT = currT;

    voltage1 = CONTROL1.computePID(vel, setpoint, deltaT);
    voltage2 = CONTROL2.computePID(vel2, setpoint, deltaT);
  }
  


  if ((micros() - refT)/ 1.0e6 < 1)
  {
    setpoint = 50 * (micros() - refT) / 1.0e6 * RPM_TO_RADS ;
  }
  else
  {
    setpoint = 50 * RPM_TO_RADS;
  }

  //setpoint = 50 * (sin(currT / 2e6) > 0) * RPM_TO_RADS;
  //setpoint = 100 * RPM_TO_RADS;

  

  if (currT / 1.0e6 > 0 && currT / 1.0e6 < 35)
  {
    update_motor_voltage(voltage1, MotorControl1);
  }
  else
  {
    update_motor_voltage(0, MotorControl1);
  }
  delay(Ts);


  /*
    Serial.print(",");
    Serial.print("posPrev: ");Serial.print(posPrev);
    Serial.print(",");
    Serial.print("posPrev2: ");Serial.print(posPrev2);
    Serial.print(",");
    Serial.print("pos: ");Serial.print(pos);
    Serial.print(",");
    Serial.print("pos2: ");Serial.print(pos2);
    Serial.print(",");
  */

  //Serial.println();

  //Serial.print(timestamp,3);


  Serial.print("Setpoint: "); Serial.print(setpoint); // / RPM_TO_RADS);
  Serial.print(",");
  Serial.print("Volt: "); Serial.print(SUPPLY_VOLTAGE);
  Serial.print(",");
  //Serial.print("deltaT: ");Serial.print(deltaT);
  //Serial.print(",");
  Serial.print("vel: "); Serial.print(vel);
  Serial.print(",");
  //Serial.print("vel2: "); Serial.print(vel2);
  //Serial.print(",");
  Serial.print("Volt1: "); Serial.print(voltage1);
  Serial.print(",");
  //Serial.print("Volt2: "); Serial.print(voltage2);
  Serial.println();


  //timestamp += Ts/1000;

}


//*****************************************************//
void update_motor_voltage(double voltage, BTU7960 motor) {
  //Fix direction based on +/-
  if (voltage < 0) {
    motor.forwardSpeed(volts_to_PWM(abs(voltage)));
  }
  else {
    motor.reverseSpeed(volts_to_PWM(abs(voltage)));
  }
}


//*****************************************************//
// Voltage and PWM duty cycle conversion
int volts_to_PWM(double voltage) {

  // Convert voltage to PWM duty cycle percent relative to the power supply voltage
  return constrain(round((voltage / SUPPLY_VOLTAGE) * 255), 0, 255);
}

double PWM_to_volts(int PWM) {
  return double(PWM / 255) * SUPPLY_VOLTAGE;
}

//Encoder Count to Radians
double count_to_deg(double count) {
  return (double(count / PPR) * 360);  // rad = (count / pulse/rev) * 360 deg/rev
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
  else { //When B falls
    if (valB == HIGH) {
      count++;  //CW
    }
    else {
      count--; //CCW
    }
  }


  if ((count > LONG_LENGTH - 1000) || (count < -LONG_LENGTH + 1000))
  {
    posPrev = posPrev - count_to_deg(count) * DEG_TO_RAD;
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
    posPrev = posPrev - count_to_deg(count) * DEG_TO_RAD;
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
    posPrev2 = posPrev2 - count_to_deg(count2) * DEG_TO_RAD;
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
    posPrev2 = posPrev2 - count_to_deg(count2) * DEG_TO_RAD;
    count2 = 0;

  }
}
