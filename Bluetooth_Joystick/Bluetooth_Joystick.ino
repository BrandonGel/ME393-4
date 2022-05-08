//Author: TableTop Robotics


int dataIn[5] {0,0,0,0}; //array to store all the information. 255,button,X,Y.
int in_byte = 0;
#define START_BUTTON 1
#define STOP_BUTTON 2
int array_index = 0;


void setup() 
{
  Serial.begin (9600); // starts the serial monitor
  Serial.println("Program Running");
 
}

void loop() {
  // put your main code here, to run repeatedly:
if (Serial.available() > 0) {  //recieve byte from phone
  in_byte = Serial.read(); //store in byte into a variable
  if (in_byte == (255)) { // if the variable is 0 stet the array inxed to 0. this will make sure that every number goes into the correct index
    array_index = 0;
  }

   if(array_index == 1)
    {
        switch(in_byte)
        {
          case START_BUTTON:
            //START();
            break;
          case STOP_BUTTON:
            //STOP();
            break;
        } 
    }
  
  dataIn[array_index] = in_byte;  //store number into array
  array_index = array_index +1;
  Serial.println (in_byte);
  
}
if(array_index == 4)
{

  //print the array
  Serial.print (dataIn[0]);
  Serial.print (", button:");
  Serial.print (dataIn[1]);
  Serial.print (", X:");
  Serial.print (dataIn[2]);
  Serial.print (", Y:");
  Serial.println (dataIn[3]);
  
}
}
