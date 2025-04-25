#ifndef CHESSBOT_SPLINES_H
#define CHESSBOT_SPLINES_H

#include <string>
#include <queue>
#include <tuple>

static std::queue<std::tuple<float, float>> timeSlicesToExecute;

void velocityUpdateTimerFunction(std::string id);
void danceMonkeyQaudratic(std::string id, Position start, Position control, Position end, float totalTime);
void danceMonkeyCubic(std::string id, Position start, Position control1, Position control2, Position end, float totalTime);

#endif