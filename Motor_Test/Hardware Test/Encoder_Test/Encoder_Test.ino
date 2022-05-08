/* 
 *  Name: Brandon Ho
 *  Date: 12/23/2021
 *  Title: Encoder Test
 *  Description: This scripts works to test the encoder tick, using interrupt function. 
 *               Meant for arduino + motor board alone, without the python GUI
 *               Using B port on the Arduino Motor Shield at full power 
 *               Print out the tick 
 *               Adapted from https://github.com/CooperControlsLab/CU-At-Home_feedback_controls/tree/master/hardware_tests/encoder_test
 */

#define ENC_A 2 // Encoder A
#define ENC_B 3 // Encoder B

int count = 0; // This variable will increase or decrease depending on the rotation of encoder

//Interrupt Functios
void pulseA();
void pulseB();


void setup() {
  Serial.begin (19200);
  pinMode(ENC_A, INPUT_PULLUP); // internally pullup encoder input A
  pinMode(ENC_B, INPUT_PULLUP); // internally pullup encoder input B

  //Setting up interrupt
  //When rotating the encoder, the encoder pin values changes.
  //Thus, the interrupt function will activate in the hardware itself
  attachInterrupt(digitalPinToInterrupt(ENC_A), pulseA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), pulseB, CHANGE);
}
   
void loop() {
  delay(100);
  Serial.println(count);
  delay(10);
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
