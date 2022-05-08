#ifndef BTU7960_h
#define BTU7960_h

#include "Arduino.h"


class BTU7960
{
  public:
    BTU7960(int pin1, int pin2){
      RPWM_Output = pin1;
      LPWM_Output = pin2;
      pinMode(RPWM_Output, OUTPUT);
      pinMode(LPWM_Output, OUTPUT);
      };

    void forwardSpeed(int PWM);    
    void reverseSpeed(int PWM);    
      
  private:
    int RPWM_Output = 0; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
    int LPWM_Output = 0; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
};

#endif
