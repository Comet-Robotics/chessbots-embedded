#include "robot/trapezoidalProfileNew.h"
#include "utils/config.h"

TrapezoidProfile::State TrapezoidProfile::calculate(double t, const State &current, const State &goal)
{
    int m_direction = shouldFlipAcceleration(current, goal) ? -1 : 1;
    State m_current = direct(current, m_direction);
    State goalDir = direct(goal, m_direction);

    if (m_current.velocity > m_constraints.maxVelocity)
    {
        m_current.velocity = m_constraints.maxVelocity;
    }
    else if (m_current.velocity < -m_constraints.maxVelocity)
    {
        m_current.velocity = -m_constraints.maxVelocity;
    }

    double cutoffBegin = m_current.velocity / m_constraints.maxAcceleration;
    double cutoffDistBegin = cutoffBegin * cutoffBegin * m_constraints.maxAcceleration / 2.0;

    double cutoffEnd = max((goalDir.velocity - MIN_MOTOR_VELOCITY_TPS), 0.0)  / m_constraints.maxAcceleration;
    double cutoffDistEnd = cutoffEnd * cutoffEnd * m_constraints.maxAcceleration / 2.0;

    double fullTrapezoidDist = cutoffDistBegin + (goalDir.position - m_current.position) + cutoffDistEnd;
    double accelerationTime = m_constraints.maxVelocity / m_constraints.maxAcceleration;

    double fullSpeedDist = fullTrapezoidDist - accelerationTime * accelerationTime * m_constraints.maxAcceleration;

    if (fullSpeedDist < 0)
    {
        accelerationTime = std::sqrt(fullTrapezoidDist / m_constraints.maxAcceleration);
        fullSpeedDist = 0;
    }

    double m_endAccel = accelerationTime - cutoffBegin;
    double m_endFullSpeed = m_endAccel + fullSpeedDist / m_constraints.maxVelocity;
    double m_endDecel = m_endFullSpeed + accelerationTime - cutoffEnd;
    State result(m_current.position, m_current.velocity);

    if (t < m_endAccel)
    {
        result.velocity += t * m_constraints.maxAcceleration;
        if (abs(result.velocity) < MIN_MOTOR_VELOCITY_TPS) {
            result.velocity = MIN_MOTOR_VELOCITY_TPS;
        }
        result.position += (m_current.velocity + t * m_constraints.maxAcceleration / 2.0) * t;
    }
    else if (t < m_endFullSpeed)
    {
        result.velocity = m_constraints.maxVelocity;
        result.position +=
            (m_current.velocity + m_endAccel * m_constraints.maxAcceleration / 2.0) * m_endAccel +
            m_constraints.maxVelocity * (t - m_endAccel);
    }
    else if (t <= m_endDecel)
    {
        double timeLeft = m_endDecel - t;
        result.velocity = goalDir.velocity + timeLeft * m_constraints.maxAcceleration;
        result.position = goalDir.position - (goalDir.velocity * timeLeft) - (timeLeft * timeLeft * m_constraints.maxAcceleration / 2.0);
    }
    else
    {
        result = goalDir;
    }

    if (abs(result.position - goalDir.position) <= 100) {
        result.velocity = 0;
    }

    return direct(result, m_direction);
}
