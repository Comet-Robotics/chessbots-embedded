#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDcontroller {
public:
    //PIDcontroller(double kp, double ki, double kd, double dt, double min, double max);
    PIDcontroller(double kp, double ki, double kd, double dt, double min, double max){
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->dt = dt;
        this->minOutput = min;
        this->maxOutput = max;
        this->prev_error = 0;
        this->integral = 0;
    }
        double  Compute(double setpoint, double actual_value, double dt);
        void    Reset(); //Extra space between void and reset because I can't see uneven indent and spacing after doing python for a long time

    double kp, ki, kd;      // PID gains
    double dt;              // Time step
    double minOutput, maxOutput;  // Output limits

    double prev_error;      // Previous error
    double integral;        // Integral term
    double derivative;      // Derivative term
};

#endif // PID_CONTROLLER_H
