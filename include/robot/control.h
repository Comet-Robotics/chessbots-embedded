#ifndef CHESSBOT_CONTROL_H
#define CHESSBOT_CONTROL_H

#include "Arduino.h"
#include "utils/config.h"
#include "robot/trapezoidalProfile.h"

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
static double headingTarget = 0.0;

static MotionProfile profileA = {THEORETICAL_MAX_VELOCITY_TPS, THEORETICAL_MAX_ACCELERATION_TPSPS, 0, 0, 0, 0, 75.0}; // maxVelocity, maxAcceleration, currentPosition, currentVelocity, targetPosition, targetVelocity
static MotionProfile profileB = {THEORETICAL_MAX_VELOCITY_TPS, THEORETICAL_MAX_ACCELERATION_TPSPS, 0, 0, 0, 0, 75.0}; // maxVelocity, maxAcceleration, currentPosition, currentVelocity, targetPosition, targetVelocity

void setLeftMotorControl(ControlSetting control);
void setRightMotorControl(ControlSetting control);
void setHeadingTarget(double target);
ControlSetting getLeftMotorControl();
ControlSetting getRightMotorControl();
double getHeadingTarget();

void setupBot();
void drive(float tiles, std::string id);
void drive(float leftPower, float rightPower, std::string id);
void driveTicks(int tickDistance, std::string id);
void turn(float angleRadians, std::string id);
void stop();
void sendPacketOnPidComplete(std::string id);
void readLight(int loopDelayMs);
void startMotorAndEncoderTest();
bool checkMoveFinished();
void startDriveTest();
void startCentering();
bool checkIfCanUpdateMovement();
void resetSpeed();
void createDriveUntilNewTile();
void determineNextChoice();
uint8_t driveUntilNewTile();
void driveTestOff();
boolean isRobotPidAtTarget();
void updateCritRange();
void controlLoop(int loopDelayMs, int8_t framesUntilPrint);
void updateCentering();
void updateToNextDistance();
void testDPRGNav();
std::tuple<bool, int, int> navigate();
void sense_location(double leftVelocityTicks, double rightVelocityTicks, int deltaTime);
void locate_target();
#endif