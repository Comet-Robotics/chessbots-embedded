#ifndef TRAPEZOIDAL_PROFILE_NEW_H
#define TRAPEZOIDAL_PROFILE_NEW_H

#include <stdexcept>
#include <cmath>

class TrapezoidProfile
{
public:
    struct Constraints
    {
        double maxVelocity;
        double maxAcceleration;

        Constraints(double maxVelocity, double maxAcceleration)
        {
            if (maxVelocity < 0.0 || maxAcceleration < 0.0)
            {
                throw std::runtime_error("Constraints must be non-negative");
            }
            this->maxVelocity = maxVelocity;
            this->maxAcceleration = maxAcceleration;
            // Remove MathSharedStore.reportUsage for now (Java-specific)
        }
    };

    struct State
    {
        double position = 0.0;
        double velocity = 0.0;

        State() = default;
        State(double position, double velocity)
            : position(position), velocity(velocity) {}

        bool operator==(const State& rhs) const
        {
            return position == rhs.position && velocity == rhs.velocity;
        }
    };

    TrapezoidProfile(const Constraints& constraints)
        : m_constraints(constraints), m_direction(1), m_endAccel(0), m_endFullSpeed(0), m_endDecel(0) {}

    State calculate(double t, const State& current, const State& goal);

    double timeLeftUntil(double target);

    double totalTime() const { return m_endDecel; }

    bool isFinished(double t) const { return t >= totalTime(); }

private:
    static bool shouldFlipAcceleration(const State& initial, const State& goal)
    {
        return initial.position > goal.position;
    }

    State direct(const State& in) const
    {
        return State(in.position * m_direction, in.velocity * m_direction);
    }

    int m_direction;
    Constraints m_constraints;
    State m_current;
    double m_endAccel;
    double m_endFullSpeed;
    double m_endDecel;
};

#endif