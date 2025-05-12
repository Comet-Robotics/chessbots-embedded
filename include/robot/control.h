#ifndef CHESSBOT_CONTROL_H
#define CHESSBOT_CONTROL_H

#include "Arduino.h"

void setupBot();
void drive(float tiles);
void drive(float leftPower, float rightPower, std::string id);
void driveTicks(int tickDistanceLeft, int tickDistanceRight, std::string id);
void turn(float angleRadians, std::string id);
void stop();
void readLight(int loopDelayMs);
void startMotorAndEncoderTest();
bool checkMoveFinished();
void startDriveTest();
void createDriveUntilNewTile(bool goingForward);
void determineNextChoice();
uint8_t driveUntilNewTile();
void driveTestOff();
void controlLoop(int loopDelayMs, int8_t framesUntilPrint);
void updateCentering();
void updateToNextDistance(bool goingForward);

#endif