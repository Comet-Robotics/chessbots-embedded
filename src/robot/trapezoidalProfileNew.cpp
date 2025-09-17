#include "robot/trapezoidalProfileNew.h"

TrapezoidProfile::State TrapezoidProfile::calculate(double t, const State &current, const State &goal)
{
    m_direction = shouldFlipAcceleration(current, goal) ? -1 : 1;
    m_current = direct(current);
    State goalDir = direct(goal);

    if (std::abs(m_current.velocity) > m_constraints.maxVelocity)
    {
        m_current.velocity = std::copysign(m_constraints.maxVelocity, m_current.velocity);
    }

    double cutoffBegin = m_current.velocity / m_constraints.maxAcceleration;
    double cutoffDistBegin = cutoffBegin * cutoffBegin * m_constraints.maxAcceleration / 2.0;

    double cutoffEnd = goalDir.velocity / m_constraints.maxAcceleration;
    double cutoffDistEnd = cutoffEnd * cutoffEnd * m_constraints.maxAcceleration / 2.0;

    double fullTrapezoidDist = cutoffDistBegin + (goalDir.position - m_current.position) + cutoffDistEnd;
    double accelerationTime = m_constraints.maxVelocity / m_constraints.maxAcceleration;

    double fullSpeedDist = fullTrapezoidDist - accelerationTime * accelerationTime * m_constraints.maxAcceleration;

    if (fullSpeedDist < 0)
    {
        accelerationTime = std::sqrt(fullTrapezoidDist / m_constraints.maxAcceleration);
        fullSpeedDist = 0;
    }

    m_endAccel = accelerationTime - cutoffBegin;
    m_endFullSpeed = m_endAccel + fullSpeedDist / m_constraints.maxVelocity;
    m_endDecel = m_endFullSpeed + accelerationTime - cutoffEnd;
    State result(m_current.position, m_current.velocity);

    if (t < m_endAccel)
    {
        result.velocity += t * m_constraints.maxAcceleration;
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
        result.velocity = goalDir.velocity + (m_endDecel - t) * m_constraints.maxAcceleration;
        double timeLeft = m_endDecel - t;
        result.position =
            goalDir.position - (goalDir.velocity + timeLeft * m_constraints.maxAcceleration / 2.0) * timeLeft;
    }
    else
    {
        result = goalDir;
    }

    return direct(result);
}

double TrapezoidProfile::timeLeftUntil(double target)
{
    double position = m_current.position * m_direction;
    double velocity = m_current.velocity * m_direction;

    double endAccel = m_endAccel * m_direction;
    double endFullSpeed = m_endFullSpeed * m_direction - endAccel;

    if (target < position)
    {
        endAccel = -endAccel;
        endFullSpeed = -endFullSpeed;
        velocity = -velocity;
    }

    endAccel = std::max(endAccel, 0.0);
    endFullSpeed = std::max(endFullSpeed, 0.0);

    const double acceleration = m_constraints.maxAcceleration;
    const double deceleration = -m_constraints.maxAcceleration;

    double distToTarget = std::abs(target - position);
    if (distToTarget < 1e-6)
    {
        return 0;
    }

    double accelDist = velocity * endAccel + 0.5 * acceleration * endAccel * endAccel;

    double decelVelocity;
    if (endAccel > 0)
    {
        decelVelocity = std::sqrt(std::abs(velocity * velocity + 2 * acceleration * accelDist));
    }
    else
    {
        decelVelocity = velocity;
    }

    double fullSpeedDist = m_constraints.maxVelocity * endFullSpeed;
    double decelDist;

    if (accelDist > distToTarget)
    {
        accelDist = distToTarget;
        fullSpeedDist = 0;
        decelDist = 0;
    }
    else if (accelDist + fullSpeedDist > distToTarget)
    {
        fullSpeedDist = distToTarget - accelDist;
        decelDist = 0;
    }
    else
    {
        decelDist = distToTarget - fullSpeedDist - accelDist;
    }

    double accelTime =
        (-velocity + std::sqrt(std::abs(velocity * velocity + 2 * acceleration * accelDist))) / acceleration;

    double decelTime =
        (-decelVelocity + std::sqrt(std::abs(decelVelocity * decelVelocity + 2 * deceleration * decelDist))) / deceleration;

    double fullSpeedTime = fullSpeedDist / m_constraints.maxVelocity;

    return accelTime + fullSpeedTime + decelTime;
}