#include "Arduino.h"
#include "Bluetooth.h"

#include <AltSoftSerial.h>

AltSoftSerial mySerial;

void Bluetooth::init(int baudrate = 9600)
{
  mySerial.flush();
  mySerial.begin(baudrate);
}

int Bluetooth::receive()
{
  int command = 0;
  if (mySerial.available() > 0)
  {


    //recieve byte from phone
    Incoming_value = mySerial.read();      //Read the incoming data and store it into variable Incoming_value
    //Serial.print("Incoming Value: ");
    //Serial.println(Incoming_value);        //Print Value of Incoming_value in Serial monitor
    
  
    if (Incoming_value == (255) or array_index > 2)
    { // if the variable is 0 stet the array inxed to 0. this will make sure that every number goes into the correct index
      array_index = 0;
    }
    dataIn[array_index] = Incoming_value;  //store number into array
    
//      Serial.print(array_index);
//      Serial.print(" ");
//      Serial.println(Incoming_value);

    if (array_index == 1 && Incoming_value != oldButton && (Incoming_value == 1 or Incoming_value == 2))
    {
//      Serial.println("-----------------");
//      Serial.println(Incoming_value);
//      Serial.println(oldButton);
      oldButton = Incoming_value;
      switch (Incoming_value)
      {
        case START_BUTTON:
          //START();
          command = 1;
          array_index = array_index + 1;
//          Serial.println("Charging");
          return command;
          break;
        case STOP_BUTTON:
          //STOP();
          command = 2;
          array_index = array_index + 1;
//          Serial.println("NOT Charging");
          return command;
          break;
      }
      
    }
  
  array_index = array_index + 1;      
  }

  dir = dataIn[2];

  //Serial.println(dataIn[3]);

  if (dir == 1)
  {
    //turn_right(120,120);
    Serial.println("Turning Right");
    command = 3;
  }
  else if (dir == 2)
  {
    //turn_left(120,120);
    Serial.println("Turning Left");
    command = 4;
  }
  else if (dir == 3)
  {
    //driveForward(120,120);
    Serial.println("Forward");
    command = 5;
  }
  else if (dir == 4)
  {
    //turn_back(120,120);
    Serial.println("Backward");
    command = 6;
  }
  else
  {
    //stay();
//    Serial.println("STOPPING");
    command = 0;
  }
//      Serial.println(command);
  
  return command;
}
