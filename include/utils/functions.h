#pragma once

#include <Arduino.h>

double fmap(double x, double in_min, double in_max, double out_min, double out_max);
bool approxEquals(int x, int y, int epsilon);
bool approxEquals(double x, double y, double epsilon);
int radiansToTicks(double angle);
std::string unint8ArrayToHexString(uint8_t* oldArray, int len);