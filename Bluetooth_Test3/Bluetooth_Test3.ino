
//With Josh's board

#define START_BUTTON 1
#define STOP_BUTTON 2


int Incoming_value = 0;
int dataIn[3] {0, 0, 0}; //array to store all the information. 255,button,X,Y.
int array_index = 0;
int oldButton = 0;
int state = 0;
int dir = 0;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);         //Sets the data rate in bits per second (baud) for serial data transmission
  Serial1.begin(9600);
}



void loop() {
  // put your main code here, to run repeatedly:
  int command = 0;
  if (Serial1.available() > 0)
  {


    //recieve byte from phone
    Incoming_value = Serial1.read();      //Read the incoming data and store it into variable Incoming_value

    //Serial.print("Incoming Value: ");
    //Serial.println(Incoming_value);        //Print Value of Incoming_value in Serial monitor

    dir = dataIn[2];
    //  Serial.print(array_index);
    //  Serial.print(" ");
    //  Serial.println(Incoming_value);
    //Serial.println(dataIn[3]);
    //Serial.println(posX);
    //Serial.println(posY);
    if (state == 1)
    {
      if (dir == 1)
      {
        //turn_right(120,120);
        command = 3;
      }
      else if (dir == 2)
      {
        //turn_left(120,120);
        command = 4;
      }
      else if (dir == 3)
      {
        //driveForward(120,120);
        command = 5;
      }
      else if (dir == 4)
      {
        //turn_back(120,120);
        command = 6;
      }
      else
      {
        //stay();
        command = 0;
      }
    }
    if (Incoming_value == (255))
    { // if the variable is 0 stet the array inxed to 0. this will make sure that every number goes into the correct index
      array_index = 0;
    }



    if (array_index == 1 && Incoming_value != oldButton)
    {
      switch (Incoming_value)
      {
        case START_BUTTON:
          //START();
          command = 1;
          state = 1;
          break;
        case STOP_BUTTON:
          //STOP();
          command = 2;
          state = 0;
          break;
      }
      oldButton = Incoming_value;
    }


    dataIn[array_index] = Incoming_value;  //store number into array
    array_index = array_index + 1;
  }



  Serial.print(array_index);
  Serial.print(" ");
  Serial.print(Incoming_value);
  Serial.print(" ");
  Serial.print(state);
  Serial.print(" ");
  Serial.println(command);
  delay(10);
}
