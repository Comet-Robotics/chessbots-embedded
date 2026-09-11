#pragma once

class PIDController
{
public:
    // PIDController(double kp, double ki, double kd, double min, double max);
    PIDController(double kp, double ki, double kd, double min, double max, double errorTolerance);

    double Compute(double setpoint, double actual_value, double dt);
    void Reset();

    double kp, ki, kd;           // PID gains
    double minOutput, maxOutput; // Output limits

    double prev_error, prev_velocity_error; // Previous error
    double errorTolerance; // Allowed error before returning 0
    double integral;   // Integral accumulator

protected:
    virtual double getError(double setpoint, double actual_value) { return setpoint - actual_value; }
};

class ContinuousPIDController : public PIDController
{
public:
    ContinuousPIDController(double kp, double ki, double kd, double min, double max, double errorTolerance, double minInput, double maxInput)
        : PIDController(kp, ki, kd, min, max, errorTolerance), minInput(minInput), maxInput(maxInput) {}
protected:
    double getError(double setpoint, double actual_value) override;
private:
    double minInput, maxInput; // Input range for continuous wrapping
};