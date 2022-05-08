#ifndef PID_h
#define PID_h

#include "Arduino.h"



class PID
{
  public:
    PID(double Ts0 = 5, double sigma0 = 0.01, double kp0 = 1, double ki0 =0.5, double kd0 = 0){
      Ts = Ts0;
      sigma = sigma0;
      beta = (2.0 * sigma - Ts/1000) / (2.0 * sigma + Ts/1000);
      kp = kp0;
      ki = ki0;
      kd = kd0;
      };
    //PID constants
    double kp;
    double ki;
    double kd;

    double u = 0;
    
    double sigma; //dirty derivative bandwidth = 1/sigma
    double beta; //(2.0*sigma-Ts)/(2.0*sigma+Ts)
    double Ts; //(2.0*sigma-Ts)/(2.0*sigma+Ts)

    double lowerOutputLimit = 0;
    double upperOutputLimit = 0; // Controller params

    
    void init();
    void updateParameters(double kp0, double ki0, double kd0);
    void readLimits(double low, double high);
    double computePID(double inp, double setpoint, double deltaT);
    
  private:
    double timestamp = 0;
    double error = 0;
    double lastError = 0;
    double cumError = 0;
    double rateError = 0;

};





#endif
