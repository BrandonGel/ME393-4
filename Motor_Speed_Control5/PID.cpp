#include "Arduino.h"
#include "PID.h"




void PID::readLimits(double low, double high){
  lowerOutputLimit = low;
  upperOutputLimit = high; // Controller params
}

void PID::updateParameters(double kp0, double ki0, double kd0){
  kp = kp0;
  ki = ki0;
  kd = kd0;
}


double PID::computePID(double inp, double setpoint, double deltaT) 
{
  error = setpoint - inp;                                // determine error
  cumError += (error + lastError) / 2 * deltaT;            // compute integral
  rateError = beta * rateError + (1 - beta)* (error - lastError)/ deltaT ; // compute derivative
  Serial.println(error);
  Serial.println(cumError);
  Serial.println(rateError);
  double u_unsat = kp * error + ki * cumError + kd * rateError;          //PID output
  u = constrain(u_unsat, lowerOutputLimit, upperOutputLimit); //return the unsaturated signal

  // Integrator anti-windup 
  if (ki > 0)
  {
    cumError += (u-u_unsat)/ki;
  }
  
  lastError = error;                                //remember current error
  return u;                                        //have function return the PID output
}
