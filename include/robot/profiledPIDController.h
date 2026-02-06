#ifndef PROFILED_PID_CONTROLLER_H
#define PROFILED_PID_CONTROLLER_H

#include "robot/pidController.h"
#include "robot/trapezoidalProfileNew.h"

class ProfiledPIDController {
public:
    ProfiledPIDController(double kp, double ki, double kd,
                          double minOutput, double maxOutput,
                          double errorTolerance,
                          const TrapezoidProfile::Constraints& constraints)
        : pid(kp, ki, kd, minOutput, maxOutput, errorTolerance), profile(constraints), lastTime(0.0) {}

    // Call this every control loop
    double Compute(double goalPosition, double actualPosition, double actualVelocity, double dt) {
        TrapezoidProfile::State current(actualPosition, actualVelocity);
        TrapezoidProfile::State goal(goalPosition, 0.0); // Assume goal velocity is zero

        // Generate profile for current time
        TrapezoidProfile::State profiledSetpoint = profile.calculate(lastTime, current, goal);

        // PID tracks profiled position
        double output = pid.Compute(profiledSetpoint.position, actualPosition, dt);

        lastTime += dt;
        return output;
    }

    void Reset() {
        pid.Reset();
        lastTime = 0.0;
    }

private:
    PIDController pid;
    TrapezoidProfile profile;
    double lastTime;
};

#endif // PROFILED_PID_CONTROLLER_H