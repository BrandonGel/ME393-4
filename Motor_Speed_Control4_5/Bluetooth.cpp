//#include "Arduino.h"
//#include "Bluetooth.h"
//#include <SoftwareSerial.h>
//
//SoftwareSerial mySerial(5, 6); // RX, TX
//
//void Bluetooth::init(int baudrate = 9600)
//{
//  mySerial.flush();
//  mySerial.begin(baudrate);
//}
//
//int Bluetooth::receive()
//{
//  int command = 0;
//  if (mySerial.available() > 0)
//  {
//
//
//    //recieve byte from phone
//    Incoming_value = mySerial.read();      //Read the incomifng data and store it into variable Incoming_value
//    //Serial.print("Incoming Value: ");
//    //Serial.println(Incoming_value);        //Print Value of Incoming_value in Serial monitor
//    dataIn[array_index] = Incoming_value;  //store number into array
//  
//  
//    if (Incoming_value == (255) && array_index > 2)
//    { // if the variable is 0 stet the array inxed to 0. this will make sure that every number goes into the correct index
//      array_index = 0;
//    }
//
//    if (array_index == 1 && Incoming_value != oldButton)
//    {
//      switch (Incoming_value)
//      {
//        case START_BUTTON:
//          //START();
//          command = 1;
//          array_index = array_index + 1;
//          return command;
//          break;
//        case STOP_BUTTON:
//          //STOP();
//          command = 2;
//          array_index = array_index + 1;
//          return command;
//          break;
//      }
//      oldButton = Incoming_value;
//    }
//
//array_index = array_index + 1;
//
//    
//  }
//
//  dir = dataIn[2];
//
//  //Serial.println(dataIn[3]);
//
//  if (dir == 1)
//  {
//    //turn_right(120,120);
//    command = 3;
//  }
//  else if (dir == 2)
//  {
//    //turn_left(120,120);
//    command = 4;
//  }
//  else if (dir == 3)
//  {
//    //driveForward(120,120);
//    command = 5;
//  }
//  else if (dir == 4)
//  {
//    //turn_back(120,120);
//    command = 6;
//  }
//  else
//  {
//    //stay();
//    command = 0;
//  }
//      Serial.print(array_index);
//      Serial.print(" ");
//      Serial.println(Incoming_value);
//      Serial.println(command);
//  
//  return command;
//}
