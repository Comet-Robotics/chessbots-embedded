#ifndef CHESSBOT_SPLINES_H
#define CHESSBOT_SPLINES_H

#include <string>
#include <queue>
#include <tuple>

static std::queue<std::tuple<float, float>> timeSlicesToExecute;

void velocityUpdateTimerFunction(std::string id);

#endif