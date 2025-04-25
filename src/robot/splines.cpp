#ifndef CHESSBOT_SPLINES_CPP
#define CHESSBOT_SPLINES_CPP

#include "robot/splines.h"
#include "robot/control.h"
#include "utils/timer.h"
#include "wifi/connection.h"
#include <tuple>

void velocityUpdateTimerFunction(std::string id)
{
    if (timeSlicesToExecute.size() == 0) {
        if (id != "NULL")
        {
            createAndSendPacket(2, "success", id);
        }
        return;
    }

    float desiredVelocityLeft, desiredVelocityRight;
    std::tie(desiredVelocityLeft, desiredVelocityRight) = timeSlicesToExecute.front();
    leftMotorControl = { VELOCITY, desiredVelocityLeft };
    rightMotorControl = { VELOCITY, desiredVelocityRight };
    timeSlicesToExecute.pop();

    // 1ms delay ensures the function will run in the next loop
    timerDelay(1, [id](){ velocityUpdateTimerFunction(id); });
}

#endif