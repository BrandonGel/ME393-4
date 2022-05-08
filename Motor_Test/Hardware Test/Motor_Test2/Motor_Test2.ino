/* 
 *  Name: Brandon Ho
 *  Date: 12/23/2021
 *  Title: Motor Test2
 *  Description: This scripts works to test the motor speed in going a cyclic motion from 0 power to full power to 0 power in one direction
 *               then 0 to full power to 0 in the other direction. Then repeat.
 *               Meant for arduino + motor board alone, without the python GUI
 *               Using B port on the Arduino Motor Shield at full power 
 *               Print out the tick 
 *               Adapted from https://github.com/CooperControlsLab/CU-At-Home_feedback_controls/tree/master/hardware_tests/encoder_test
 */

#define ENC_A 2 // Encoder A
#define ENC_B 3 // Encoder B

//Parameter for using Channel B motor based on Arduino Motor Shield
#define PWM_B 11 //Speed B
#define BRK_B 8 //Brake B
#define DIR_B 13 //Direction B


int count = 0; // This variable will increase or decrease depending on the rotation of encoder

//Interrupt Functios
void pulseA();
void pulseB();


void setup() {
  Serial.begin (19200);
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
}

int PWM = 0;
int inc = 0; //Track the number of changes in PWM
void loop() {

  //First Half cycle for B in the CW
  digitalWrite(DIR_B, LOW); //CW for B
  cycle(1); //increase the speed of B from 0 to 255 by 1
  cycle(-1); //decrease the speed of B from 255 to 0 by 1

  //Second Half cycle for B in the CW
  digitalWrite(DIR_B, HIGH); //CCW for B
  cycle(1); //increase the speed of B from 0 to 255 by 1
  cycle(-1); //decrease the speed of B from 255 to 0 by 1

}

//Either increase the motor speed from 0 to 255
// or decrease the motor speed from 255 to 0
void cycle(int inc)
{
  for(int ii = 0; ii < 256; ii++)
  {
    analogWrite(PWM_B,PWM);
    double cur_time = micros();
    double time_stamp = micros();
    while(time_stamp-cur_time < 1000){
      Serial.print(time_stamp);
      Serial.print(",");
      Serial.print(count);
      Serial.print(",");
      Serial.println(PWM);
      time_stamp = micros();
    }
    PWM = PWM + inc;
  }
}

//Encoder Interrupt A
void pulseA()
{
  bool valA = digitalRead(ENC_A);
  bool valB = digitalRead(ENC_B);

  if(valA == HIGH){ //When A rises
    if(valB == LOW){
      count++; //CW
    }
    else{
      count--; //CCW
    }
  }
  else{ //When B falls
    if(valB == HIGH){
      count++;  //CW
    }
    else{
      count--; //CCW
    }
  }
}


//Encoder Interrupt B
void pulseB()
{
  bool valA = digitalRead(ENC_A);
  bool valB = digitalRead(ENC_B);

  if(valB == HIGH) //When B rises
  {
    if(!valA == LOW){
      count++; //CW
    }
    else{
      count--; //CCW
    }
  }
  else //When B falls
  {
    if(valA == HIGH){
      count++; //CW
    }
    else{
      count--; //CCW
    }
  }
}
