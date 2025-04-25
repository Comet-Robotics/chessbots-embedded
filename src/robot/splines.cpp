#ifndef CHESSBOT_SPLINES_CPP
#define CHESSBOT_SPLINES_CPP

#include "robot/splines.h"
#include "utils/logging.h"
#include "robot/control.h"
#include "utils/timer.h"
#include "wifi/connection.h"
#include "utils/config.h"
#include <tuple>
#include <queue>



void velocityUpdateTimerFunction(std::string id)
{
    serialLogln("Update Timer was called", 3);
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


void danceMonkeyCubic(std::string id, Point start, Point control1, Point control2, Point end, float totalTime){
    float trackWidth = TRACK_WIDTH_INCHES;

    int steps = (int)(totalTime / 0.01); // 10ms steps
    float dt = totalTime / steps;

    for (int i = 0; i < steps; i++) {
        float t = (float)i / steps;
        float tNext = (float)(i + 1) / steps;

        // Cubic Bezier Point at t
        Point p = {
            (float)(pow(1 - t, 3) * start.x +
                    3 * pow(1 - t, 2) * t * control1.x +
                    3 * (1 - t) * pow(t, 2) * control2.x +
                    pow(t, 3) * end.x),
            (float)(pow(1 - t, 3) * start.y +
                    3 * pow(1 - t, 2) * t * control1.y +
                    3 * (1 - t) * pow(t, 2) * control2.y +
                    pow(t, 3) * end.y)
        };

        // Cubic Bezier Point at tNext
        Point pNext = {
            (float)(pow(1 - tNext, 3) * start.x +
                    3 * pow(1 - tNext, 2) * tNext * control1.x +
                    3 * (1 - tNext) * pow(tNext, 2) * control2.x +
                    pow(tNext, 3) * end.x),
            (float)(pow(1 - tNext, 3) * start.y +
                    3 * pow(1 - tNext, 2) * tNext * control1.y +
                    3 * (1 - tNext) * pow(tNext, 2) * control2.y +
                    pow(tNext, 3) * end.y)
        };

        // Estimate linear velocity
        float dx = pNext.x - p.x;
        float dy = pNext.y - p.y;
        float linearVelocity = sqrt(dx * dx + dy * dy) / dt;

        // Estimate angular velocity
        float theta1 = atan2(dy, dx);
        float theta0;

        if (i > 0) {
            float tPrev = (float)(i - 1) / steps;
        Point pPrev = {
                (float)(pow(1 - tPrev, 3) * start.x +
                        3 * pow(1 - tPrev, 2) * tPrev * control1.x +
                        3 * (1 - tPrev) * pow(tPrev, 2) * control2.x +
                        pow(tPrev, 3) * end.x),
                (float)(pow(1 - tPrev, 3) * start.y +
                        3 * pow(1 - tPrev, 2) * tPrev * control1.y +
                        3 * (1 - tPrev) * pow(tPrev, 2) * control2.y +
                        pow(tPrev, 3) * end.y)
            };

            theta0 = atan2(p.y - pPrev.y, p.x - pPrev.x);
        } else {
            theta0 = theta1;
        }

        float angularVelocity = (theta1 - theta0) / dt;

        // Normalize angle diff
        while (angularVelocity > M_PI) angularVelocity -= 2 * M_PI;
        while (angularVelocity < -M_PI) angularVelocity += 2 * M_PI;

        // Compute wheel velocities using differential drive model
        float vLeft = linearVelocity - (angularVelocity * trackWidth / 2);
        float vRight = linearVelocity + (angularVelocity * trackWidth / 2);

        timeSlicesToExecute.push({vLeft, vRight});
    }
    velocityUpdateTimerFunction(id);
}

void danceMonkeyQaudratic(std::string id, Point start, Point control, Point end, float totalTime) {
    serialLogln("Quadratic Dancer was called", 3);
    float trackWidth = TRACK_WIDTH_INCHES;
    int steps = (int)(totalTime / 0.01); // 10ms steps
    float dt = totalTime / steps;

    for (int i = 0; i < steps; i++) {
        float t = (float)i / steps;
        float tNext = (float)(i + 1) / steps;

        // Bezier point at t
        Point p = {
                (float)((1 - t) * (1 - t) * start.x + 2 * (1 - t) * t * control.x + t * t * end.x),
                (float)((1 - t) * (1 - t) * start.y + 2 * (1 - t) * t * control.y + t * t * end.y)
        };

        // Bezier point at t + dt
        Point pNext = {
            (float)((1 - tNext) * (1 - tNext) * start.x + 2 * (1 - tNext) * tNext * control.x + tNext * tNext * end.x),
            (float)((1 - tNext) * (1 - tNext) * start.y + 2 * (1 - tNext) * tNext * control.y + tNext * tNext * end.y)
        };

        // Velocity approximation
        float dx = pNext.x - p.x;
        float dy = pNext.y - p.y;
        float linearVelocity = sqrt(dx * dx + dy * dy) / dt;

        // Angular velocity approximation
        float theta1 = atan2(dy, dx);
        float theta0 = (i > 0) ? atan2(p.y - ((1 - (float)(i - 1) / steps) * (1 - (float)(i - 1) / steps) * start.y +
            2 * (1 - (float)(i - 1) / steps) * ((float)(i - 1) / steps) * control.y +
            ((float)(i - 1) / steps) * ((float)(i - 1) / steps) * end.y),
            p.x - ((1 - (float)(i - 1) / steps) * (1 - (float)(i - 1) / steps) * start.x +
            2 * (1 - (float)(i - 1) / steps) * ((float)(i - 1) / steps) * control.x +
            ((float)(i - 1) / steps) * ((float)(i - 1) / steps) * end.x)) : theta1;
        float angularVelocity = (theta1 - theta0) / dt;

        // Normalize angle diff
        while (angularVelocity > M_PI) angularVelocity -= 2 * M_PI;
        while (angularVelocity < -M_PI) angularVelocity += 2 * M_PI;

        // Differential drive kinematics
        float vLeft = linearVelocity - (angularVelocity * trackWidth / 2);
        float vRight = linearVelocity + (angularVelocity * trackWidth / 2);


        timeSlicesToExecute.push({vLeft, vRight});
        serialLogln(vLeft, 3);
        serialLogln(vRight, 3);
    }
    velocityUpdateTimerFunction(id);
}



#endif