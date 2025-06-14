#ifndef CHESSBOT_FUNCTIONS_H
#define CHESSBOT_FUNCTIONS_H

// Built-In Libraries
#include "Arduino.h"

float fmap(float x, float in_min, float in_max, float out_min, float out_max);
bool approxEquals(int x, int y, int epsilon);
bool approxEquals(double x, double y, double epsilon);
int radiansToTicks(double angle);
std::string unint8ArrayToHexString(uint8_t* oldArray, int len);

#endif