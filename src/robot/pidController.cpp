#ifndef PID_CONTROLLER_CPP
#define PID_CONTROLLER_CPP

#include "robot/pidController.h"
#include <iostream>
#include <cmath>
#include <algorithm>

double PIDController::Compute(double setpoint, double actual_value, double dt) {
    // Calculate error
    double error = setpoint - actual_value;

    if(std::abs(error) < 2) {
        return 0;
    }

    // Proportional term
    double val_p = kp * error; //We will be messing with this while calibrating

    // Integral term
    integral += error * dt;
    double val_i = ki * integral;
 
    // Derivative term
    double derivative = (error - prev_error) / dt;
    double val_d = kd * derivative;

    // Calculate total output
    double output = val_p + val_i + val_d;

    // Save error to previous error
    prev_error = error;

    if ((prev_error > 0 && error < 0) || (prev_error < 0 && error > 0)) {
        Reset();
    }

    // Clamp output
    output = std::max(minOutput, std::min(maxOutput, output));

    return output;
}

void PIDController::Reset(){
    integral = 0;
}

#endif