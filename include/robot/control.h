#ifndef CHESSBOT_CONTROL_H
#define CHESSBOT_CONTROL_H

#include "Arduino.h"

void setupBot();
void drive(float tiles);
void drive(float leftPower, float rightPower, std::string id);
void stop();
void readLight(uint8_t counter, int prevTickVals[], const uint8_t TICK_DIST);
void startDriveTest();
void driveTestOff();

#endif