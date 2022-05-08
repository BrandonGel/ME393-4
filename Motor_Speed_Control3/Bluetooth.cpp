#include "Arduino.h"
#include "Bluetooth.h"

int Bluetooth::receive()
{
  int command = 0;
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
            //START();
            command = 1;
            break;
          case STOP_BUTTON:
            //STOP();
            command = 2;
            break;
        } 
        oldButton = Incoming_value;
    }
    
    
    dataIn[array_index] = Incoming_value;  //store number into array
    array_index = array_index +1;
  }
  
  dir = dataIn[2];
  Serial.println(dataIn[0]);
  Serial.println(dataIn[1]);
  Serial.println(dataIn[2]);
  Serial.println(dataIn[3]);
  //Serial.println(posX);
  //Serial.println(posY);
  if(dir == 1)
  {
    //turn_right(120,120);
    command = 3;
  }
  else if(dir == 2)
  {
    //turn_left(120,120);
    command = 4;
  }
  else if(dir == 3)
  {
    //driveForward(120,120);
    command = 5;
  }
  else if(dir == 4)
  {
    //turn_back(120,120);
    command = 6;
  }
  else
  {
    //stay();
    command = 0;
  }
  return command;
}
