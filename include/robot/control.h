#ifndef CHESSBOT_CONTROL_H
#define CHESSBOT_CONTROL_H

#include "Arduino.h"

void setupBot();
void drive(float tiles);
void drive(float leftPower, float rightPower, std::string id);
void stop();
void readLight(bool* onFirstTile);
void startDriveTest();
void driveTestOff();

#endif