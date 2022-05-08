/* 
 *  Name: Brandon Ho
 *  Date: 12/23/2021
 *  Title: Encoder Test
 *  Description: This scripts works to test the motor from going 0 power to full power. Then stop.
 *               Meant for arduino + motor board alone, without the python GUI
 *               Using B port on the Arduino Motor Shield at full power 
 *               Print out the tick 
 *               Adapted from https://github.com/CooperControlsLab/CU-At-Home_feedback_controls/tree/master/hardware_tests/motor_test/motor_test_arduino
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

  
  if(inc <= 255)
  {
    digitalWrite(DIR_B, LOW); //CW for B
    analogWrite(PWM_B,PWM);
    long cur_time = micros();
    long time_stamp = micros();
    while(time_stamp-cur_time < 1000){
      Serial.print(time_stamp);
      Serial.print(",");
      Serial.print(count);
      Serial.print(",");
      Serial.println(PWM);
      time_stamp = micros();
    }
    inc++;
    PWM++;
  }
  else{
    digitalWrite(PWM_B,LOW);
    delay(100);
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
