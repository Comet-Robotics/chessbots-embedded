#ifndef PID_CONTROLLER_CPP
#define PID_CONTROLLER_CPP

#include "robot/pidController.h"
#include "utils/logging.h"
#include <iostream>
#include <cmath>
#include <algorithm>

double PIDController::Compute(double setpoint, double actual_value, double dt) {
    // Calculate error
    double error = this->getError(setpoint, actual_value);

    if (abs(error) < errorTolerance) {
        return 0;
    }

    // Proportional term
    double val_p = kp * error; //We will be messing with this while calibrating
    if(val_p != val_p){
        if(error > 0){
            val_p = 2147483647;
        } else {
            val_p = -2147483647;
        }
        Reset();
    }

    // Integral term
    integral += error * dt;
    double val_i = ki * integral;
    if(val_i != val_i){
        if(integral > 0){
            val_i = 2147483647;
        } else{
            val_i = -2147483647;
        }
        Reset();
    }
 
    // Derivative term
    double derivative = (error - prev_error) / dt;
    double val_d = kd * derivative;
    if(val_d != val_d){
        if(derivative > 0){
            val_d = 2147483647;
        } else{
            val_d = -2147483647;
        }
        Reset();
    }

    // Calculate total output
    double output = val_p + val_i + val_d;
    if(output != output){
        if(error > 0){
            val_d = 2147483647;
        } else{
            val_d = -2147483647;
        }
        Reset();
    }

    // If error and prev error have different signs, reset integral accumulator
    if ((prev_error > 0 && error < 0) || (prev_error < 0 && error > 0)) {
        Reset();
    }

    // Save error to previous error
    prev_error = error;

    // Clamp output
    output = std::max(minOutput, std::min(maxOutput, output));

    // serialLog("PID is outputting: ", 3);
    // serialLog(float(output), 3);
    // serialLogln(",", 3);
    // if (abs(output) < 0.2) {
    //     output = 0;
    // }

    return (output);
}

void PIDController::Reset(){
    integral = 0;
}

#endif