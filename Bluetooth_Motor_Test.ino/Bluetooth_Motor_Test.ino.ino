
//Motor Shield Pin Values
const int leftDirection = 12;
const int leftSpeed = 3;
const int leftBrake = 9;
const int rightDirection = 13;
const int rightSpeed = 11;
const int rightBrake = 8;


//Bluetooth Values
int Incoming_value = 0;                //Variable for storing Incoming_value
int dataIn[4] {0,0,0,0}; //array to store all the information. 255,button,X,Y.
#define START_BUTTON 1
#define STOP_BUTTON 2
int array_index = 0;



void setup() 
{
  
    
  //Initialize the Serial Monitor
  delay(200);
  Serial.begin(9600);
  Serial.println("Program Running");

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

  Serial.flush();
  delay(200);
}




void driveForward(int left_speed_value, int right_speed_value) {
  analogWrite(leftSpeed, left_speed_value);
  analogWrite(rightSpeed, right_speed_value);
  digitalWrite(leftDirection, HIGH);
  digitalWrite(rightDirection, HIGH);
  //drive_straight();
}

void turn_right(int left_speed_value, int right_speed_value) {
  analogWrite(leftSpeed, left_speed_value);
  analogWrite(rightSpeed, right_speed_value);
  digitalWrite(leftDirection, LOW);
  digitalWrite(rightDirection, HIGH);
  Serial.println("Right");
}

void turn_left(int left_speed_value, int right_speed_value) {
  analogWrite(leftSpeed, left_speed_value);
  analogWrite(rightSpeed, right_speed_value);
  digitalWrite(leftDirection, HIGH);
  digitalWrite(rightDirection, LOW);
  Serial.println("Left");

}

void turn_back(int left_speed_value, int right_speed_value) {
  analogWrite(leftSpeed, left_speed_value);
  analogWrite(rightSpeed, right_speed_value);
  digitalWrite(leftDirection, LOW);
  digitalWrite(rightDirection, LOW);
  Serial.println("Back");
}


void stay() {
  analogWrite(leftSpeed, 0);
  analogWrite(rightSpeed, 0);
  digitalWrite(leftDirection, LOW);
  digitalWrite(rightDirection, LOW);
  Serial.println("stay");
}


void STOP() {
  digitalWrite(leftBrake, HIGH);   //brake A
  digitalWrite(rightBrake, HIGH);   //brake B
  Serial.println("STOP Motors!");
  delay(200);
}

void START() {
  digitalWrite(leftBrake, LOW);   //brake A
  digitalWrite(rightBrake, LOW);   //brake B
  Serial.println("Starting Motors!");
}

void motor_control(int left, int right) {
  digitalWrite(leftDirection, left >= 0);
  digitalWrite(rightDirection, right >= 0);
  analogWrite(leftSpeed, left);
  analogWrite(rightSpeed, right);
}


int oldButton = 0;

void loop() 
{
  
  if (Serial.available() > 0) 
  {  
    //recieve byte from phone
    Incoming_value = Serial.read();      //Read the incoming data and store it into variable Incoming_value
    //Serial.print("Incoming Value: ");
    //Serial.println(Incoming_value);        //Print Value of Incoming_value in Serial monitor

     
    if (Incoming_value == (255)) 
    { // if the variable is 0 stet the array inxed to 0. this will make sure that every number goes into the correct index
      array_index = 0;
    }

    if(array_index == 1 && Incoming_value != oldButton)
    {
        switch(Incoming_value)
        {
          case START_BUTTON:
            START();
            break;
          case STOP_BUTTON:
            STOP();
            break;
        } 
        oldButton = Incoming_value;
    }
    
    
    dataIn[array_index] = Incoming_value;  //store number into array
    array_index = array_index +1;
  }

      

  int posX = dataIn[2];
  int posY = dataIn[3];
  Serial.println(posX);
  Serial.println(posY);
  if(150 < posX && posX < 250 && -posX+250 < posY && posY < posX)
  {
    turn_right(120,120);
  }
  else if(0 < posX && posX < 100 && posX < posY && posY < -posX+250)
  {
    turn_left(120,120);
  }
  else if(posY < posX && posX < -posY+250 && 0 < posY && posY < 100)
  {
    driveForward(120,120);
  }
  else if(-posY+250 < posX && posX < posY && 150 < posY && posY < 250)
  {
    turn_back(120,120);
  }
  else
  {
    stay();
  }

}
