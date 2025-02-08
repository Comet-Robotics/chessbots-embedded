#ifndef CHESSBOT_FUNCTIONS_H
#define CHESSBOT_FUNCTIONS_H

// Built-In Libraries
#include "Arduino.h"

namespace ChessBot
{
    float fmap(float x, float in_min, float in_max, float out_min, float out_max);
    std::string unint8ArrayToHexString(uint8_t* oldArray, int len);
};

#endif