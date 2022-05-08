
#ifndef Bluetooth_h
#define Bluetooth_h

//Bluetooth Values
//Variable for storing Incoming_value

#define START_BUTTON 1
#define STOP_BUTTON 2




class Bluetooth{
  public:
    Bluetooth(){};
    void init(int baudrate);
    int receive();
    int dir = 0;
    
    
    
    
  private:
    
    int Incoming_value = 0; 
    int dataIn[3] {0,0,0}; //array to store all the information. 255,button,X,Y.
    int array_index = 0;
    int oldButton = 0;
    int state = 0;
};




#endif
