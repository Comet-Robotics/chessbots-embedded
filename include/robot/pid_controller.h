#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController
{
public:
    // PIDController(double kp, double ki, double kd, double min, double max);
    PIDController(double kp, double ki, double kd, double min, double max) : kp(kp), ki(ki), kd(kd), minOutput(min), maxOutput(max), prev_error(0), integral(0) {}

    double Compute(double setpoint, double actual_value, double dt);
    void Reset(); // Extra space between void and reset because I can't see uneven indent and spacing after doing python for a long time

    double kp, ki, kd;           // PID gains
    double minOutput, maxOutput; // Output limits

    double prev_error; // Previous error
    double integral;   // Integral accumulator
};

#endif // PID_CONTROLLER_H
