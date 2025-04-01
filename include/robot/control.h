#ifndef CHESSBOT_CONTROL_H
#define CHESSBOT_CONTROL_H

#include "Arduino.h"

void setupBot();
void drive(float tiles);
void drive(float leftPower, float rightPower, std::string id);
void stop();
void readLight();
void startMotorAndEncoderTest();
void startDriveTest();
void driveTestOff();
void driveDistance(int distance);
void controlLoop(int loopDelayMs);

#endif