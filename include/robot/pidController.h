#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController
{
public:
    // PIDController(double kp, double ki, double kd, double min, double max);
    PIDController(double kp, double ki, double kd, double min, double max, double errorTolerance)
        : kp(kp), ki(ki), kd(kd), minOutput(min), maxOutput(max), errorTolerance(errorTolerance), prev_error(0), integral(0) {}

    double Compute(double setpoint, double actual_value, double dt);
    void Reset();

    double kp, ki, kd;           // PID gains
    double minOutput, maxOutput; // Output limits

    double prev_error, prev_velocity_error; // Previous error
    double integral;   // Integral accumulator
    double errorTolerance; // Allowed error before returning 0
protected:
    virtual double getError(double setpoint, double actual_value) { return setpoint - actual_value; }
};
// Manually making the clamp function
// template <typename T>
// T clamp(T value, T minValue, T maxValue) {
//     return (value < minValue) ? minValue : (value > maxValue) ? maxValue : value;
// }

class ContinuousPIDController : public PIDController
{
public:
    ContinuousPIDController(double kp, double ki, double kd, double min, double max, double errorTolerance, double minInput, double maxInput)
        : PIDController(kp, ki, kd, min, max, errorTolerance), minInput(minInput), maxInput(maxInput) {}
protected:
    double getError(double setpoint, double actual_value) override {
        double error = setpoint - actual_value;
        double range = maxInput - minInput;
        if (error > range / 2) {
            error -= range;
        } else if (error < -range / 2) {
            error += range;
        }
        return error;
    }
private:
    double minInput, maxInput; // Input range for continuous wrapping
};

#endif // PID_CONTROLLER_H
