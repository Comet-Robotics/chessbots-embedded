#ifndef CHESSBOT_CONTROL_H
#define CHESSBOT_CONTROL_H

#include "Arduino.h"

enum OperatingMode
{
    POSITION,
    VELOCITY
};

struct ControlSetting
{
    OperatingMode mode;
    float value;
};

static ControlSetting leftMotorControl;
static ControlSetting rightMotorControl;

void setupBot();
void drive(float tiles);
void drive(float leftPower, float rightPower, std::string id);
void driveTicks(int tickDistance, std::string id);
void turn(float angleRadians, std::string id);
void stop();
void readLight();
void startMotorAndEncoderTest();
void startDriveTest();
void driveTestOff();
void controlLoop(int loopDelayMs);

#endif