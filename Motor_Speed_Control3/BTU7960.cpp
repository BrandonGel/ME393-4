#include "Arduino.h"
#include "BTU7960.h"


void BTU7960::forwardSpeed(int PWM)
{  
  analogWrite(LPWM_Output, 0);
  analogWrite(RPWM_Output, PWM);
}

void BTU7960::reverseSpeed(int PWM)
{
  analogWrite(LPWM_Output, PWM);
  analogWrite(RPWM_Output, 0);
}
