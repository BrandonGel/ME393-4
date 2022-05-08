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
//#include "Bluetooth.h"
#include "PinChangeInterrupt.h"

#include "LCD.h" // includes the LiquidCrystal Library 
LCD lcdScreen;


// Define hardware pins for Motor Control system
#define ENC1_A 19   //Encoder pulse A
#define ENC1_B 18   //Encoder pulse B
const int ENC2_A = A11;   //Encoder pulse A
const int ENC2_B = A12;   //Encoder pulse B

#define PPR 464.64 //Encoder pulses per revolution
//#define PPR_A 11*34 //Pulse A per r





#define RAD_TO_DEG 565.486677646
#define DEG_TO_RAD 0.01745329251
#define RPM_TO_RADS 0.104719755
#define DEGS_TO_RPM 0.166667
#define PI 3.1415926535897932384626433832795

#define LONG_LENGTH 2147483647
#define MOTOR_RES 12.6 // in [Ω]
#define WHEEL_DIA 0.080 // in [m]
#define AXLE_LENGTH 0.304 // in [m]
//Bluetooth BT;
MPU6050 IMU;

// dirty derivative inputs for D control
double Ts = .01; //sample period in milliseconds
double sigma = 0.05; //sample period in milliseconds


// bandwidth inputs for velocity
double sigmaA = 0.2; //sample period in milliseconds
double betaA = (2.0 * sigmaA - Ts) / (2.0 * sigmaA + Ts); //sample period in milliseconds;

// bandwidth inputs for yaw
double sigma2 = 0.2; //sample period in milliseconds


PID CONTROL(Ts, sigma, 1, .2, 0.001);
PID CONTROL_yaw(Ts, sigma2, 1, 0, 0);


//Global VariablesUnsat
double SUPPLY_VOLTAGE;

const int leftDirection = 12;
const int leftSpeed = 3;
const int leftBrake = 9;
const int rightDirection = 13;
const int rightSpeed = 11;
const int rightBrake = 8;


//Interrupt Functios
void pulse1A();
void pulse1B();
void pulse2A();
void pulse2B();


float vel_D;
float yaw_D;
float yawDot_D;
long count = 0; // This variable will increase or decrease depending on the rotation of encoder
long count2 = 0; // This variable will increase or decrease depending on the rotation of encoder


double voltage_yawDot;
double voltage1;
double voltage1_enc;
double voltage2;
double voltage2_enc;

double current1;
double current2;

float pos = 0;
float pos2 = 0;
float posPrev = 0;
float posPrev2 = 0;
long prevT = 0;
float deltaT = 0;
float vel1 = 0;
float vel2 = 0;
float vel = 0;
float yaw = 0;
float yawDot = 0;

long refT;
int command = 0;
int state = 0;




void setup() {
  //Initialize the Serial Monitor
  //delay(200);
  Serial.flush();
  //delay(200);
  Serial.begin(19200);
  //BT.init(19200);
  //Serial.println("Program Running");

  lcdScreen.init(10000,10000,A1);




  pinMode(ENC1_A, INPUT_PULLUP); // internally pullup encoder input A
  pinMode(ENC1_B, INPUT_PULLUP); // internally pullup encoder input B
  pinMode(ENC2_A, INPUT_PULLUP); // internally pullup encoder input A
  pinMode(ENC2_B, INPUT_PULLUP); // internally pullup encoder input B

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

  //Setting up interrupt
  //When rotating the encoder, the encoder pin values changes.
  //Thus, the interrupt function will activate in the hardware itself
  attachInterrupt(digitalPinToInterrupt(ENC1_A), pulse1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), pulse1B, CHANGE);
  attachPCINT(digitalPinToPCINT(ENC2_A), pulse2A, CHANGE);
  attachPCINT(digitalPinToPCINT(ENC2_B), pulse2B, CHANGE);

  //Initalize the Motor Speed & Direction

  IMU.init();
  //Serial.println("Vel_D:,Supply_Volt:,vel1:,vel2:,Volt1:,Volt2:");
  //Serial.println("Vel_D:,Yaw_D,YawDot_D,Supply_Volt:,vel1:,Yaw:,YawDot:,Volt1:,I1:");
  Serial.println("SetVel_D:,YawDot_D,Supply_Volt:,vel1:,vel2:,YawDot:,Volt1:,Volt2:");
  delay(500);
  refT = micros();

}


 float setpoint_vel = 50*RPM_TO_RADS;
float setpoint_yawD = 0;
float oldcommand = 0;
void loop()
{
//  command = BT.receive();
//
//  switch (command)
//  {
//    case 0: //Stay
//      vel_D = 0;
//      yaw_D = 0;
//      yawDot_D  = 0;
//      break;
//    case 1: //Start
//      vel_D = 0;
//      yaw_D = 0;
//      yawDot_D  = 0;
//      state = 1;
//      break;
//    case 2: //Stop
//      vel_D = 0;
//      yaw_D = 0;
//      yawDot_D  = 0;
//      state = 0;
//
//      break;
//    case 3: // Turn right
//      vel_D = 0;
//      yaw_D = -50 * RPM_TO_RADS;
//      yawDot_D  = 0;
//
//      break;
//    case 4: // Turn left
//      vel_D = 0;
//      yaw_D = 50 * RPM_TO_RADS;
//      yawDot_D  = 0;
//
//      break;
//    case 5: // Forward
//      vel_D = 50 * RPM_TO_RADS;
//      yaw_D = 0;
//      yawDot_D  = 0;
//
//      break;
//    case 6: // Backward
//      vel_D = -50 * RPM_TO_RADS;
//      yaw_D = 0;
//      yawDot_D  = 0;
//
//      break;
//  }
//  if (oldcommand != command) {
//    refT = micros();
//    oldcommand = command;
//  }
  
  if ((micros() - refT) / 1.0e6 < 1)
  {
    setpoint_vel = 50 * (micros() - refT) / 1.0e6 * RPM_TO_RADS ;
    setpoint_yawD = 0;
  }
  else
  {
    setpoint_vel = 50 * RPM_TO_RADS;
    setpoint_yawD = 0;
  }

  //vel_D = 50 * (sin(currT / 2e6) > 0) * RPM_TO_RADS;
  //vel_D = 100 * RPM_TO_RADS;

  //vel_D = 50* RPM_TO_RADS;
  if (state == 0)
  {

    //SUPPLY_VOLTAGE = 6;
    SUPPLY_VOLTAGE = lcdScreen.displayVolt();
    CONTROL.readLimits(-SUPPLY_VOLTAGE, SUPPLY_VOLTAGE);
    //CONTROL.readLimits(0, SUPPLY_VOLTAGE);

    pos = count_to_deg(count) * DEG_TO_RAD; // in [rad]
    pos2 = count_to_deg(count2) * DEG_TO_RAD; // in [rad]

    long currT = micros(); // in [us]
    deltaT = ((float) (currT - prevT)) / 1.0e6; // in [s]
    if (deltaT >= Ts)
    {
      vel1 = betaA * vel1 + (1 - betaA) / deltaT * (pos - posPrev); // in [rad/s]
      vel2 = betaA * vel2 + (1 - betaA) / deltaT * (pos2 - posPrev2); // in [rad/s]
      vel = (vel1 + vel2)/2;
      posPrev = pos;
      posPrev2 = pos2;
      prevT = currT;
      
      
      voltage1_enc = CONTROL.computePID(vel1, setpoint_vel, deltaT);
      voltage2_enc = CONTROL.computePID(vel2, setpoint_vel, deltaT);
      //voltage = CONTROL.computePID(vel, vel_D, deltaT);

      
      IMU.getAcc();
      yaw = IMU.yaw;//*DEG_TO_RAD;
      
      double yawDot_IMU = IMU.GyroZ * DEG_TO_RAD;
      double yawDot_enc = (vel1-vel2) * WHEEL_DIA * AXLE_LENGTH / 2;
//      yawDot = yawDot_IMU;
//      double yawDot_enc = (vel1-vel2)*WHEEL_DIA*AXLE_LENGTH/4;
      double yawDot = sensorFusion(yawDot_enc, yawDot_IMU, .5);
      voltage_yawDot = 10*CONTROL_yaw.computePID(yawDot, setpoint_yawD, deltaT);
      Serial.println(yawDot_IMU);
      Serial.println(yawDot_enc);
      Serial.println(voltage_yawDot);
      Serial.println(yawDot);
      voltage1 = voltage1_enc - voltage_yawDot;
      voltage2 = voltage2_enc + voltage_yawDot;

      voltage1 = constrain(voltage1, -SUPPLY_VOLTAGE, SUPPLY_VOLTAGE);
      voltage2 = constrain(voltage2, -SUPPLY_VOLTAGE, SUPPLY_VOLTAGE);


      //current1 = voltage1 / MOTOR_RES;
    }


    //if (currT / 1.0e6 > 0 && currT / 1.0e6 < 35)
    if (currT / 1.0e6 > 0)
    {
      update_motor_voltage(voltage2, rightDirection, rightSpeed);
      update_motor_voltage(voltage1, leftDirection, leftSpeed);
    }
    else
    {
      update_motor_voltage(0, rightDirection, rightSpeed);
      update_motor_voltage(0, leftDirection, leftSpeed);
    }

    Serial.print(setpoint_vel); // / RPM_TO_RADS);
    Serial.print(",");
    //Serial.print(yaw_D); // / RPM_TO_RADS);
    //Serial.print(",");
    Serial.print(yawDot_D); // / RPM_TO_RADS);
    Serial.print(",");
    Serial.print(SUPPLY_VOLTAGE);
    Serial.print(",");
    Serial.print(vel1);
    Serial.print(",");
    Serial.print(vel2);
    Serial.print(",");
    //Serial.print(yaw); // / RPM_TO_RADS);
    //Serial.print(",");
    Serial.print(yawDot); // / RPM_TO_RADS);
    Serial.print(",");
    

    Serial.print(voltage1);
    Serial.print(",");
    Serial.println(voltage2);
    //Serial.print(current1);
//    Serial.println();
    

  }
  else
  {
    //Serial.println("Press Start to run");

  }
  //  Serial.println(state);
  //  Serial.println(command);
  delay(Ts*1000);
}


//*****************************************************//
void update_motor_voltage(double voltage, int DIR, int PWM) {
  //Fix direction based on +/-
  if (voltage < 0) {
    digitalWrite(DIR, LOW); // CCW
  }
  else {
    digitalWrite(DIR, HIGH); // CW
  }
  //set pwm based off of voltage
  analogWrite(PWM, volts_to_PWM(abs(voltage)));
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
