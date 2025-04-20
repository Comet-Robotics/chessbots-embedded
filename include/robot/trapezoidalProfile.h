struct MotionProfile {
    double maxVelocity;
    double maxAcceleration;
    double currentPosition;
    double targetPosition;
    double currentVelocity;
    double requiredVelocity;
};

double updateTrapezoidalProfile(MotionProfile &profile, double dt);

template <typename T>
T clamp(T value, T minValue, T maxValue) {
    return (value < minValue) ? minValue : (value > maxValue) ? maxValue : value;
}