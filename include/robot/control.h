#ifndef CHESSBOT_CONTROL_H
#define CHESSBOT_CONTROL_H

#include "Arduino.h"

void setupBot();
void drive(float tiles);
void drive(float leftPower, float rightPower, std::string id);
void driveTicks(int tickDistance, std::string id);
void turn(float angleRadians, std::string id);
void stop();
void readLight();
void startMotorAndEncoderTest();
void startDriveTest();
void createDriveUntilNewTile();
uint8_t driveUntilNewTile();
void driveTestOff();
void controlLoop(int loopDelayMs);
void updateCentering();

#endif