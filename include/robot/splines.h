#ifndef CHESSBOT_SPLINES_H
#define CHESSBOT_SPLINES_H

#include <string>
#include <queue>
#include <tuple>
#include "wifi/packet.h"
#include "trapezoidalProfile.h"

struct Point {
    float x;
    float y;
};

static std::queue<std::tuple<float, float>> timeSlicesToExecute;

void velocityUpdateTimerFunction(std::string id);
void danceMonkeyQuadratic(std::string id, Point start, Point control, Point end, float totalTime);
void danceMonkeyCubic(std::string id, Point start, Point control1, Point control2, Point end, float totalTime);
void startCustomMotionProfileTimer(int leftPositionTarget, int rightPositionTarget, double profileDuration, std::string id);
void customMotionProfileTimerFunction(MotionProfile &customProfileA, MotionProfile &customProfileB, double dt, std::string id);

#endif