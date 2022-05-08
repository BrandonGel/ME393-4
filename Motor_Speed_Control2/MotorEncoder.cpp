
#include "Arduino.h"
#include "MotorEncoder.h"








//Encoder Count to Radians
double MotorEncoder::count_to_deg(double count) {
  return (double(count / PPR) * 360);  // rad = (count / pulse/rev) * 360 deg/rev
}

//Encoder Interrupt 1A
void MotorEncoder::pulse1A()
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
void MotorEncoder::pulse1B()
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

//Setting up interrupt
  //When rotating the encoder, the encoder pin values changes.
  //Thus, the interrupt function will activate in the hardware itself
void MotorEncoder::init(){
  pinMode(ENC1_A, INPUT_PULLUP); // internally pullup encoder input A
  pinMode(ENC1_B, INPUT_PULLUP); // internally pullup encoder input B
  
}
