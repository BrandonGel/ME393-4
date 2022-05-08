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

  if (ki > 0)
  {
    double unsatIntegrator = ki * cumError;
    double cumError = constrain(unsatIntegrator, lowerOutputLimit, upperOutputLimit) / ki ;
  }

  rateError = beta * rateError + (1 - beta) / deltaT * (error - lastError); // compute derivative

  u = kp * error + ki * cumError + kd * rateError;          //PID output

  u = constrain(u, lowerOutputLimit, upperOutputLimit);
  
  
  lastError = error;                                //remember current error
  return u;                                        //have function return the PID output
}
