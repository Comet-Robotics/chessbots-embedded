#ifndef CHESSBOT_TRAPEZOIDAL_PROFILE_H
#define CHESSBOT_TRAPEZOIDAL_PROFILE_H

struct MotionProfile {
    double maxVelocity;
    double maxAcceleration;
    double currentPosition;
    double currentVelocity;
    double targetPosition;
    double targetVelocity;
};

double updateTrapezoidalProfile(MotionProfile &profile, double dt);

template <typename T>
T clamp(T value, T minValue, T maxValue) {
    return (value < minValue) ? minValue : (value > maxValue) ? maxValue : value;
}

#endif