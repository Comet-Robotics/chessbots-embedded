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
#//define PID_CONTROLLER_CPP

#include "robot/pid_controller.h"

#include <iostream>
#include <cmath>
#include "robot/pid_controller.h"

using namespace std;

class PIDcontroller{
    public:
        PIDcontroller(double kp, double ki, double kd, double max, double min);
        double Compute(double setpoint, double pv);
        void Reset();
    private:
        double kp_;
        double ki_;
        double kd_;
        double max_;
        double min_;
        double pre_error_;
        double integral_;
};

PIDcontroller::PIDcontroller(double kp, double ki, double kd, double max, double min) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    max_ = max;
    min_ = min;
    pre_error_ = 0;
    integral_ = 0;
}