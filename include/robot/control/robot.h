#ifndef CHESSBOT_CONTROL_H
#define CHESSBOT_CONTROL_H

#include "Arduino.h"
#include "utils/config.h"
#include "robot/trapezoidalProfile.h"
#include "robot/control/lights.h"
#include "robot/control/motor.h"

enum OperatingMode
{
    POSITION,
    VELOCITY
};

enum DriveStatus {
    ONGOING,

    // The robot has reached its destination and is now reversing
    REACHED_REVERSING,

    // The robot has reached its destination, and reversing is not necessary
    REACHED
};

enum CenteringStatus {
    NOT_CENTERING,
    
    // The robot has started centering
    STARTED,

    // The robot has finished centering along axis 1, and is moving to an edge
    EDGE,

    // The robot is moving by itself
    MOVING
};

struct ControlSetting
{
    OperatingMode mode;
    float value;
};

// Unused?
// static ControlSetting leftMotorControl;
// static ControlSetting rightMotorControl;
// static double headingTarget = 0.0;

void setXControl(ControlSetting control);
void setYControl(ControlSetting control);
void setHeadingTarget(double target);
ControlSetting getLeftMotorControl();
ControlSetting getRightMotorControl();
double getHeadingTarget();

class Robot {
    public:
        Robot();

        static int batteryLevel();

        // Runs all the necessary processing for each tick of the global event loop
        void tick();
        
        void center();
        void drive(float tiles, std::string id);
        void drive(float leftPower, float rightPower, std::string id);
        void driveTicks(int tickDistance, std::string id);
        enum DriveStatus driveUntilNewTile();

        void turn(float angleRadians, std::string id);
        
        void stop();
    
    private:
        Motor left;
        Motor right;

        Light left_light;
        Light right_light;

        boolean stopped;
        
        CenteringStatus centeringStatus;

        MotionProfile profileX = {THEORETICAL_MAX_VELOCITY_TPS, THEORETICAL_MAX_ACCELERATION_TPSPS, 0, 0, 0, 0, 75.0}; // maxVelocity, maxAcceleration, currentPosition, currentVelocity, targetPosition, targetVelocity
        MotionProfile profileY = {THEORETICAL_MAX_VELOCITY_TPS, THEORETICAL_MAX_ACCELERATION_TPSPS, 0, 0, 0, 0, 75.0}; // maxVelocity, maxAcceleration, currentPosition, currentVelocity, targetPosition, targetVelocity
        MotionProfile profileA = {THEORETICAL_MAX_VELOCITY_TPS, THEORETICAL_MAX_ACCELERATION_TPSPS, 0, 0, 0, 0, 75.0}; // maxVelocity, maxAcceleration, currentPosition, currentVelocity, targetPosition, targetVelocity
        
        void center_tick();
        void turn_tick();
        void pid_tick();
        // void setLeftPower(leftPower);
        // void setRightPower(rightPower);

};

void setupBot();

void sendPacketOnPidComplete(std::string id);
void readLight(int loopDelayMs);
bool checkMoveFinished();
void startCentering();
bool checkIfCanUpdateMovement();
void resetSpeed();
void createDriveUntilNewTile();
void determineNextChoice();
uint8_t driveUntilNewTile();
boolean isRobotPidAtTarget();
void updateCritRange();
void controlLoop(int loopDelayMs);
void updateCentering();
void updateToNextDistance();

void startDriveTest();
void driveTestOff();
void startMotorAndEncoderTest();

#endif