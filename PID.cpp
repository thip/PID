#include "PID.h"


//constructor for the PIDController
PIDController::PIDController(pid_bundle coeffs){  
  this->coeffs = coeffs;
}

//updates the internals of the controller, ignores d for now
void PIDController::update(double delta, long millis){
    static double t_0 = 1;
    static double p_0 = 0;

    double t = millis/1000;

    p = delta;
    i = (i+p) * (millis/1000) * coeffs.i_decay;
    d = (p - p_0)/(t - t_0);
}

//returns the output of the controller
double PIDController::output(){
    return p * coeffs.p_coef + i * coeffs.i_coef + d * coeffs.d_coef;
}


