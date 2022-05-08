/* 
 *  Name: Brandon Ho
 *  Date: 12/23/2021
 *  Title: Motor Position Control
 *  Description: This script works to test the motor position
 *               It goes with DataTransfer.py to get the DC gain and time constant of the motor
 *               Using B port on the Arduino Motor Shield at full power 
 *               Adapted from https://github.com/CooperControlsLab/CU-At-Home_feedback_controls/tree/master/hardware_tests/encoder_test
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

//Timing Parameters for fixed interval calculations for PID and derivitave
unsigned long prev_micros = 0;
unsigned long current_micros;

//Global Variables
double lowerOutputLimit = -SUPPLY_VOLTAGE;
double upperOutputLimit = SUPPLY_VOLTAGE; // Controller params


long count = 0; // This variable will increase or decrease depending on the rotation of encoder
double setpoint = 100;
double Ts = 1; //sample period in milliseconds

//Interrupt Functios
void pulseA();
void pulseB();

//PID constantsL
double kp = 10;
double ki = 15;
double kd = 1;


double sigma = 0.01; //dirty derivative bandwidth = 1/sigma
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
//float v1Prev = 0;
//float v1Filt = 0;
float deltaT = 0;


void setup() {
  Serial.begin (19200);
  Serial.flush();
  Serial.println("Start");
  Serial.println("Position Control");
  delay(200);

  pinMode(ENC_A, INPUT_PULLUP); // internally pullup encoder input A
  pinMode(ENC_B, INPUT_PULLUP); // internally pullup encoder input B

  //Motor Setup
  pinMode(PWM_B, OUTPUT);
  pinMode(BRK_B, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  
  digitalWrite(BRK_B, LOW);   //No Brake for B
  
  beta = (2.0 * sigma - Ts/1000) / (2.0 * sigma + Ts/1000);
  
  //Setting up interrupt
  //When rotating the encoder, the encoder pin values changes.
  //Thus, the interrupt function will activate in the hardware itself
  attachInterrupt(digitalPinToInterrupt(ENC_A), pulseA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), pulseB, CHANGE);
}


void loop()
{

  pos = count_to_deg(count) * DEG_TO_RAD; // in [rad]

  long currT = micros(); // in [us]
  deltaT = ((float) (currT - prevT)) / 1.0e6; // in [s]
  //float vel = (pos - posPrev) / deltaT; // in [rad/s]
  //posPrev = pos;
  prevT = currT;

  //v1Filt = 0.4360604 * v1Filt + 0.2819698 * vel + 0.2819698 * v1Prev; // in [rad/s]
  //v1Prev = vel;

  //setpoint = 270 * (sin(currT / 1e6) > 0) * RPM_TO_RADS;
  setpoint = count_to_deg(464)*DEG_TO_RAD;


  voltage = computePID(pos);
  delay(Ts);
  update_motor_voltage(voltage);

  Serial.print(pos*RAD_TO_DEG);
  Serial.print(",");
  Serial.print(error*RAD_TO_DEG);
  Serial.print(",");
  Serial.println(voltage);
//  Serial.print("Position: "); Serial.print(count_to_deg(count));
//  Serial.print(" ");
//  Serial.print("Error: "); Serial.print(error*RAD_TO_DEG);
//  Serial.print(" ");
//  Serial.print("Cumulative Error: "); Serial.print(cumError*RAD_TO_DEG);
//  Serial.print(" ");
//  Serial.print("Voltage: "); Serial.println(voltage);

}

double computePID(double inp) {


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
