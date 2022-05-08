#ifndef MotorEncoder_h
#define MotorEncoder_h

#include "Arduino.h"



#define PPR 748 //Encoder pulses per revolution

#define MOVING_AVERAGE_SIZE 50 // Size of moving average

#define RAD_TO_DEG 360/(2*3.14159)
#define DEG_TO_RAD 2*3.14159/360
#define RPM_TO_RADS 0.104719755
#define DEGS_TO_RPM 0.166667
#define PI 3.1415926535897932384626433832795
#define LONG_LENGTH 2147483647

class MotorEncoder
{
  public:
    MotorEncoder(int A, int B){
      ENC1_A = A;
      ENC1_B = B;
      };
    int ENC1_A;
    int ENC1_B;
    double SUPPLY_VOLTAGE;
    void init();
    double count_to_deg(double count);
    void pulse1A();
    void pulse1B();
    
  private:
    
    int count;
    int posPrev;
    
};


#endif
