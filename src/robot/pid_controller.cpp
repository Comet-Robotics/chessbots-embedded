//PID Implementation [ Proportional-Integral-Derivative ]
//Mudit Upadhyay
//Tasks:
// 1. Implement the PIDController class
//      1.1 Declare public methods
//      1.2 Declare private members
// 2. Implement the PIDController::Compute() method
//      2.1 Implement the error term
//      2.2 Implement the Proportional term
//      2.3 Implement the Integral term
//      2.4 Implement the Derivative term
// 3. Implement the PIDController::Reset() method
//#ifndef PID_CONTROLLER_CPP
//#define PID_CONTROLLER_CPP

#include "robot/pid_controller.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include "robot/pid_controller.h"

using namespace std;

double PIDController::Compute(double setpoint, double actual_value, double dt) {
    // Calculate error
    double error = setpoint - actual_value;

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
    if (output < minOutput) {
        output = minOutput;
    } else if (output > maxOutput) {
        output = maxOutput;
    }

    return output;
}

void PIDController::Reset(){
    integral = 0;
}

// double PIDController::ActualController() {
    
// }