/*
    Name: Brandon Ho
    Date: 2/27/2022
    Title: Motor Speed Open Loop
    Description: This script works to test the motor in an open loop response
                 It goes with DataTransfer.py to get the DC gain and time constant of the motor
                 Using B port on the Arduino Motor Shield at full power
                 Adapted from https://github.com/CooperControlsLab/CU-At-Home_feedback_controls/tree/master/arduino_code/libraries/PID
*/

// Define hardware pins for Motor Control system
#define ENC1_A 18   //Encoder pulse A
#define ENC1_B 19   //Encoder pulse B
#define ENC2_A 20   //Encoder pulse A
#define ENC2_B 21   //Encoder pulse B

#define PPR 748 //Encoder pulses per revolution
//#define PPR_A 11*34 //Pulse A per r



#define MOVING_AVERAGE_SIZE 50 // Size of moving average

#define RAD_TO_DEG 360/(2*3.14159)
#define DEG_TO_RAD 2*3.14159/360
#define RPM_TO_RADS 0.104719755
#define DEGS_TO_RPM 0.166667
#define PI 3.1415926535897932384626433832795

#define LONG_LENGTH 2147483647
int ind = 0;



//Global Variables

double SUPPLY_VOLTAGE  = 12; //12V power supply
double lowerOutputLimit = -SUPPLY_VOLTAGE;
double upperOutputLimit = SUPPLY_VOLTAGE; // Controller params

//Motor Shield Pin Values
const int leftDirection = 12;
const int leftSpeed = 3;
const int leftBrake = 9;
const int rightDirection = 13;
const int rightSpeed = 11;
const int rightBrake = 8;

const int power = 5;
const int volt_measure = A0;

//Interrupt Functios
void pulse1A();
void pulse1B();
void pulse2A();
void pulse2B();


float setpoint;
long count = 0; // This variable will increase or decrease depending on the rotation of encoder
long count2 = 0; // This variable will increase or decrease depending on the rotation of encoder

//PID constants
double kp = 1;
double ki = 5;
double kd = .0;

double sigma = 0.01; //dirty derivative bandwidth = 1/sigma
double Ts = 1; //sample period in milliseconds
double beta; //(2.0*sigma-Ts)/(2.0*sigma+Ts)

double timestamp = 0;
double error = 0;
double lastError = 0;
double cumError = 0;
double rateError = 0;

double error2 = 0;
double lastError2 = 0;
double cumError2 = 0;
double rateError2 = 0;

double voltage1;
double voltage2;
float pos = 0;
float pos2 = 0;
float posPrev = 0;
float posPrev2 = 0;
long prevT = 0;
float deltaT = 0;
bool state = 1;

void setup() {
  //Initialize the Serial Monitor
  //delay(200);
  Serial.flush();
  //delay(200);
  Serial.begin(19200);
  //Serial.println("Program Running");

  pinMode(power, OUTPUT);
  digitalWrite(power,HIGH);
  //Motor Shield Pins
  pinMode(leftDirection, OUTPUT);
  pinMode(leftSpeed, OUTPUT);
  pinMode(leftBrake, OUTPUT);
  pinMode(rightDirection, OUTPUT);
  pinMode(rightSpeed, OUTPUT);
  pinMode(rightBrake, OUTPUT);

  pinMode(volt_measure, INPUT);


  pinMode(ENC1_A, INPUT_PULLUP); // internally pullup encoder input A
  pinMode(ENC1_B, INPUT_PULLUP); // internally pullup encoder input B
  pinMode(ENC2_A, INPUT_PULLUP); // internally pullup encoder input A
  pinMode(ENC2_B, INPUT_PULLUP); // internally pullup encoder input B


beta = (2.0 * sigma - Ts/1000) / (2.0 * sigma + Ts/1000);
  //Setting up interrupt
  //When rotating the encoder, the encoder pin values changes.
  //Thus, the interrupt function will activate in the hardware itself
  attachInterrupt(digitalPinToInterrupt(ENC1_A), pulse1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), pulse1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), pulse2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), pulse2B, CHANGE);

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
  SUPPLY_VOLTAGE = analogRead(volt_measure)*10.0/1024;
  lowerOutputLimit = -SUPPLY_VOLTAGE;
  upperOutputLimit = SUPPLY_VOLTAGE; // Controller params
  pos = count_to_deg(count) * DEG_TO_RAD; // in [rad]
  pos2 = count_to_deg(count2) * DEG_TO_RAD; // in [rad]

  long currT = micros(); // in [us]
  deltaT = ((float) (currT - prevT)) / 1.0e6; // in [s]
  float vel = (pos - posPrev) / deltaT; // in [rad/s]
  float vel2 = (pos2 - posPrev2) / deltaT; // in [rad/s]
  /*
  Serial.print("Setpoint: ");Serial.print(setpoint / RPM_TO_RADS);
  Serial.print(",");
  Serial.print("diff: ");Serial.print(pos-posPrev);
  Serial.print(",");
  Serial.print("diff2: ");Serial.print(pos2-posPrev2);
  Serial.print(",");
  */
  posPrev = pos;
  posPrev2 = pos2;
  prevT = currT;  

  //setpoint = 100 * (sin(currT / 1e6) > 0) * RPM_TO_RADS;
  setpoint = 50 * RPM_TO_RADS;
  voltage1 = computePID(vel, error, cumError, lastError, rateError);
  voltage2 = computePID(vel2, error2, cumError2, lastError2, rateError2);
  delay(Ts);
  if (currT/1.0e6 > 5 && currT/1.0e6 < 35)
  { 
    update_motor_voltage(voltage2,rightDirection, rightSpeed);
    update_motor_voltage(voltage1, leftDirection, leftSpeed);
  }
  else
  {
    update_motor_voltage(0,rightDirection, rightSpeed);
    update_motor_voltage(0, leftDirection, leftSpeed);
  }
 


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

  
  Serial.print("Setpoint: ");Serial.print(setpoint);// / RPM_TO_RADS);
  Serial.print(",");
  Serial.print("Voltage: ");Serial.print(SUPPLY_VOLTAGE);
  Serial.print(","); 
  Serial.print("deltaT: ");Serial.print(deltaT);
  Serial.print(",");
  Serial.print("vel: ");Serial.print(vel);
  Serial.print(",");
  Serial.print("vel2: ");Serial.print(vel2);
  Serial.print(",");
  Serial.print("Voltage1: ");Serial.print(voltage1);
  Serial.print(",");
  Serial.print("Voltage2: ");Serial.print(voltage2);
  Serial.println();
  //*/

  //timestamp += Ts/1000;

}


double computePID(double inp, double &error, double &cumError, double &lastError, double &rateError) 
{
  error = setpoint - inp;                                // determine error
  cumError += (error + lastError) / 2 * deltaT;            // compute integral

  if (ki > 0)
  {
    double unsatIntegrator = ki * cumError;
    double cumError = constrain(unsatIntegrator, lowerOutputLimit, upperOutputLimit) / ki ;
  }

  rateError = beta * rateError + (1 - beta) / deltaT * (error - lastError); // compute derivative

  double voltage = kp * error + ki * cumError + kd * rateError;          //PID output
  voltage = constrain(voltage, lowerOutputLimit, upperOutputLimit);
  
  lastError = error;                                //remember current error
  return voltage;                                        //have function return the PID output
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
    if (!valA == LOW) {
      count++; //CW
    }
    else {
      count--; //CCW
    }
  }
  else //When B falls
  {
    if (valA == HIGH) {
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
    if (!valA == LOW) {
      count2++; //CW
    }
    else {
      count2--; //CCW
    }
  }
  else //When B falls
  {
    if (valA == HIGH) {
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
