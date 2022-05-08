/*
    Name: Brandon Ho
    Date: 12/24/2021
    Title: Motor Speed Open Loop
    Description: This script works to test the motor in an open loop response
                 It goes with DataTransfer.py to get the DC gain and time constant of the motor
                 Using B port on the Arduino Motor Shield at full power
                 Adapted from https://github.com/CooperControlsLab/CU-At-Home_feedback_controls/tree/master/arduino_code/libraries/PID
*/

// Define hardware pins for Motor Control system
#define ENC_A 2   //Encoder pulse A
#define ENC_B 3   //Encoder pulse B
#define PPR 464.64 //Encoder pulses per revolution
#define PPR_A 232.8 //Pulse A per r

//Parameter for using Channel B motor based on Arduino Motor Shield
#define DIR_B 13 //Motor Direction HIGH = CW, LOW = CCW
#define PWM_B 11 //Motor PWM
#define BRK_B 8  //Motor Break Doesn't seem to work, avoid using

#define SUPPLY_VOLTAGE 12 //12V power supply
#define MOVING_AVERAGE_SIZE 50 // Size of moving average

#define RAD_TO_DEG 360/(2*3.14159)
#define DEG_TO_RAD 2*3.14159/360
#define RPM_TO_RADS 0.104719755
#define DEGS_TO_RPM 0.166667


#define PWM_B 11 //Speed B
#define BRK_B 8 //Brake B
#define DIR_B 13 //Direction B

#define LONG_LENGTH 2147483647
int ind = 0;


//Global Variables
double lowerOutputLimit = -SUPPLY_VOLTAGE;
double upperOutputLimit = SUPPLY_VOLTAGE; // Controller params


float setpoint;
long count = 0; // This variable will increase or decrease depending on the rotation of encoder


//Interrupt Functios
void pulseA();
void pulseB();

//PID constants
double kp = .6;
double ki = .8;
double kd = .0;

double sigma = 0.01; //dirty derivative bandwidth = 1/sigma
double Ts = 5; //sample period in milliseconds
double beta; //(2.0*sigma-Ts)/(2.0*sigma+Ts)

double timestamp = 0;
double error = 0;
double lastError = 0;
double cumError = 0;
double rateError = 0;

double voltage;
float pos = 0;
float posPrev = 0;
long prevT = 0;
float v1Prev = 0;
float v1Filt = 0;
float deltaT = 0;
bool state = 1;

void setup() {
  Serial.begin (19200);
  Serial.flush();
  Serial.println("Start");
  Serial.println("Open Loop");
  delay(200);

  pinMode(ENC_A, INPUT_PULLUP); // internally pullup encoder input A
  pinMode(ENC_B, INPUT_PULLUP); // internally pullup encoder input B

  //Motor Setup
  pinMode(PWM_B, OUTPUT);
  pinMode(BRK_B, OUTPUT);
  pinMode(DIR_B, OUTPUT);

  digitalWrite(BRK_B, LOW);   //No Brake for B



  //Setting up interrupt
  //When rotating the encoder, the encoder pin values changes.
  //Thus, the interrupt function will activate in the hardware itself
  attachInterrupt(digitalPinToInterrupt(ENC_A), pulseA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), pulseB, CHANGE);


  setpoint = 0;
  prevT = 0;

  beta = (2.0 * sigma - Ts) / (2.0 * sigma + Ts);


  Serial.print("Time ");
  Serial.print("Setpoint ");
  Serial.print("Velocity ");
  Serial.print("Error ");
  //Serial.print("Cum Error: ");
  //Serial.print(cumError/RPM_TO_RADS);
  //Serial.print(" ");
  //Serial.print("beta: ");
  //Serial.println(beta);
  //Serial.print(" ");
  Serial.println("Voltage");
}


void loop()
{

  pos = count_to_deg(count) * DEG_TO_RAD; // in [rad]

  long currT = micros(); // in [us]
  deltaT = ((float) (currT - prevT)) / 1.0e6; // in [s]
  float vel = (pos - posPrev) / deltaT; // in [rad/s]
  posPrev = pos;
  prevT = currT;  

  voltage = 12*(sin(currT / 1e6) > 0);
  update_motor_voltage(voltage);
  delay(Ts);



  Serial.print(timestamp,3);
  Serial.print(",");
  Serial.print(vel);
  Serial.print(",");
  Serial.println(voltage);

  timestamp += Ts/1000;


}


//*****************************************************//
void update_motor_voltage(double voltage) {
  //Fix direction based on +/-
  if (voltage < 0) {
    digitalWrite(DIR_B, LOW); // CCW
  }
  else {
    digitalWrite(DIR_B, HIGH); // CW
  }
  //set pwm based off of voltage
  analogWrite(PWM_B, volts_to_PWM(abs(voltage)));
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


//Encoder Interrupt A
void pulseA()
{
  bool valA = digitalRead(ENC_A);
  bool valB = digitalRead(ENC_B);

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


//Encoder Interrupt B
void pulseB()
{
  bool valA = digitalRead(ENC_A);
  bool valB = digitalRead(ENC_B);

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
