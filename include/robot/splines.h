#ifndef CHESSBOT_SPLINES_H
#define CHESSBOT_SPLINES_H

#include <string>
#include <queue>
#include <tuple>
#include "wifi/packet.h"


struct Point {
    float x;
    float y;
};

static std::queue<std::tuple<float, float>> timeSlicesToExecute;

void velocityUpdateTimerFunction(std::string id);
void danceMonkeyQaudratic(std::string id, Point start, Point control, Point end, float totalTime);
void danceMonkeyCubic(std::string id, Point start, Point control1, Point control2, Point end, float totalTime);

#endif