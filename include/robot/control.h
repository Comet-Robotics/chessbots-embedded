#ifndef CHESSBOT_CONTROL_H
#define CHESSBOT_CONTROL_H

#include "Arduino.h"

void setupBot();
void drive(float tiles);
void drive(float leftPower, float rightPower, std::string id);
void driveTicks(int tickDistance, std::string id);
void turn(float angleRadians, std::string id);
void stop();
void readLight(bool* onFirstTile);
void startMotorAndEncoderTest();
void startDriveTest();
void createDriveUntilNewTile(bool* onFirstTile);
uint8_t driveUntilNewTile(bool* onFirstTile);
void beginXTicksDrive(uint8_t leadingEncoderLabel, int max_ticks, bool inReverse);
void moveRobotXTicks();
void driveTestOff();
void controlLoop(int loopDelayMs);

#endif