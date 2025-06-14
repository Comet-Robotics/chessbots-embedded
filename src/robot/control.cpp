#ifndef CHESSBOT_CONTROL_CPP
#define CHESSBOT_CONTROL_CPP

// Associated Header File
#include "robot/control.h"
#include "robot/trapezoidalProfile.h"

// Built-In Libraries
#include "Arduino.h"
#include <queue>

// Custom Libraries
#include "utils/logging.h"
#include "utils/timer.h"
#include "utils/config.h"
#include "utils/status.h"
#include "robot/motor.h"
#include "robot/lightSensor.h"
#include "wifi/connection.h"
#include "robot/encoder.h"
#include "robot/pidController.h"
#include "robot/driveTest.h"
#include "../../env.h"
#include <algorithm>
#include "utils/functions.h"

//PLEASE ONLY USE CHESSBOT #4 FOR TESTING
PIDController encoderAVelocityController(0.00008, 0.0000035, 0.000001, -1, +1); //Blue
PIDController encoderBVelocityController(0.00008, 0.0000035, 0.000001, -1, +1); //Red

int prevPositionA = 0;
int prevPositionB = 0;

boolean testEncoderPID_value = false;
void testEncoderPID()
{
    serialLogln("Changing encoder PID setpoint!", 2);
    if (!testEncoderPID_value)
    {
        testEncoderPID_value = true;
        setLeftMotorControl({POSITION, (float)TICKS_PER_ROTATION * 5});
        setRightMotorControl({POSITION, (float)TICKS_PER_ROTATION * 5});
    }
    else
    {
        testEncoderPID_value = false;
        setLeftMotorControl({POSITION, 0.0f});
        setRightMotorControl({POSITION, 0.0f});
    }
}

int testTurn_angle = 0;
void testTurn()
{
    serialLog("Changing destination angle to ", 2);
    serialLog(testTurn_angle, 2);
    testTurn_angle = (testTurn_angle + 90) % 360;
    turn(M_PI / 2, "NULL");
    serialLog(" (", 2);
    serialLog(getLeftMotorControl().value, 2);
    serialLog(", ", 2);
    serialLog(getRightMotorControl().value, 2);
    serialLogln(")", 2);
}

// Sets up all the aspects needed for the bot to work
void setupBot() {
    serialLogln("Setting Up Bot...", 2);
    setupMotors();
    setupIR();
    setupEncodersNew();
    serialLogln("Bot Set Up!", 2);

    encoderAVelocityController.Reset();
    encoderBVelocityController.Reset();

    if (DO_PID_TEST) {
        testEncoderPID();
        timerInterval(8000, &testEncoderPID);
    }
    
    if (DO_TURN_TEST) {
        testTurn();
        timerInterval(5000, &testTurn);
    }
}

// + (0.0001 * desiredVelocityA)
// Manages control loop (loopDelayMs is for reference)
void controlLoop() {
    if (DO_LIGHT_SENSOR_TEST)
        readLight();

    if (DO_ENCODER_TEST)
        encoderLoop();

    if (DO_PID) {
        double loopDelaySeconds = ((double) loopDelayMilliseconds) / 1000;

        int currentPositionEncoderA = readLeftEncoder();
        int currentPositionEncoderB = readRightEncoder();
        double currentVelocityA = (currentPositionEncoderA - prevPositionA) / loopDelaySeconds;
        double currentVelocityB = (currentPositionEncoderB - prevPositionB) / loopDelaySeconds;

        profileA.currentPosition = currentPositionEncoderA;
        profileA.currentVelocity = currentVelocityA;
        profileB.currentPosition = currentPositionEncoderB;
        profileB.currentVelocity = currentVelocityB;

        double desiredVelocityA, desiredVelocityB;

        if (getLeftMotorControl().mode == POSITION) {
            profileA.targetPosition = getLeftMotorControl().value;
            desiredVelocityA = updateTrapezoidalProfile(profileA, loopDelaySeconds);
        } else {
            desiredVelocityA = getLeftMotorControl().value;
        }
        if (getRightMotorControl().mode == POSITION) {
            profileB.targetPosition = getRightMotorControl().value;
            desiredVelocityB = updateTrapezoidalProfile(profileB, loopDelaySeconds);
        } else {
            desiredVelocityB = getRightMotorControl().value;
        }

        prevPositionA = currentPositionEncoderA;
        prevPositionB = currentPositionEncoderB;

        double leftFeedForward = desiredVelocityA / MAX_VELOCITY_TPS;
        double rightFeedForward = desiredVelocityB / MAX_VELOCITY_TPS;

        double leftMotorPower = encoderAVelocityController.Compute(desiredVelocityA, currentVelocityA, loopDelaySeconds) + leftFeedForward;
        double rightMotorPower = encoderBVelocityController.Compute(desiredVelocityB, currentVelocityB, loopDelaySeconds) + rightFeedForward;

        if (leftMotorPower > 1) leftMotorPower = 1;
        if (leftMotorPower < -1) leftMotorPower = -1;
        if (rightMotorPower > 1) rightMotorPower = 1;
        if (rightMotorPower < -1) rightMotorPower = -1;

        // serialLog(currentPositionEncoderA, 3);
        // serialLog(",", 3);
        // serialLog(currentPositionEncoderB, 3);
        // serialLog(",", 3);
        // serialLog(desiredVelocityA, 3);
        // serialLog(",", 3);
        // serialLog(desiredVelocityB, 3);
        // serialLog(",", 3);
        // serialLog(currentVelocityA, 3);
        // serialLog(",", 3);
        // serialLog(currentVelocityB, 3);
        // serialLog(",", 3);
        // // serialLog(leftMotorPower, 3);
        // // serialLog(",", 3);
        // // serialLog(rightMotorPower, 3);
        // // serialLog(",", 3);
        // serialLog(leftMotorControl.mode == POSITION ? leftMotorControl.value : 0, 3);
        // serialLog(",", 3);
        // serialLog(rightMotorControl.mode == POSITION ? rightMotorControl.value : 0, 3); // TODO log results of trapezoidal profile into csv (on motor value graph)
        // serialLog(",", 3);
        // serialLog(isRobotPidAtTarget(), 3);
        // // serialLog(",", 3);

        // // serialLogln(loopDelaySeconds, 3);
         

        drive(
            leftMotorPower, // leftMotorPower,
            rightMotorPower, // rightMotorPower,
            "NULL"
        );

        // serialLogln(leftMotorPower, 3);

        // turn(M_PI / 2, "NULL");
    }
}

void setLeftMotorControl(ControlSetting control) {
    leftMotorControl = control;
}

void setRightMotorControl(ControlSetting control) {
    rightMotorControl = control;
}

ControlSetting getLeftMotorControl() {
    return leftMotorControl;
}

ControlSetting getRightMotorControl() {
    return rightMotorControl;
}

void drive(float tiles, std::string id) {
    if (!getStoppedStatus()) {
        const float TILE_SIZE_INCHES = 24;
        float distanceInches = tiles * TILE_SIZE_INCHES;
        float ticksPerInch = TICKS_PER_ROTATION / (WHEEL_DIAMETER_INCHES * M_PI);
        int tickDistance = (int)(distanceInches * ticksPerInch);
        driveTicks(tickDistance, id);
    }    
    
}

void driveTicks(int tickDistance, std::string id)
{
    if (!getStoppedStatus()) {
        setLeftMotorControl({POSITION, (float)(readLeftEncoder() + tickDistance)});
        setRightMotorControl({POSITION, (float)(readRightEncoder() + tickDistance)});

        if (id != "NULL") {
            sendPacketOnPidComplete(id);
        }
    }
}

// Drives the wheels according to the powers set. Negative is backwards, Positive forwards
void drive(float leftPower, float rightPower, std::string id) {
    if (!getStoppedStatus()) {
        // TODO: maybe move to motor.cpp?
        float minPower = 0.16;
        if (leftPower < minPower && leftPower > -minPower) {
            leftPower = 0;
        } if (rightPower < minPower && rightPower > -minPower) {
            rightPower = 0;
        }

        setLeftPower(leftPower);
        setRightPower(rightPower);

        //we only send null as id during our test drive. The only other time this drive method is called will be
        //when the server sends it, meaning it will have an id to send back.
        if(id != "NULL") {
            createAndSendPacket(2, "success", id);   
        }
    }
}

void turn(float angleRadians, std::string id) {

    serialLogln("Turning", 3);
    serialLogln(angleRadians, 3);
    int offsetTicks = radiansToTicks(angleRadians);

    if (getLeftMotorControl().mode == POSITION) {
        setLeftMotorControl({POSITION, getLeftMotorControl().value - offsetTicks});
    } else {
        setLeftMotorControl({POSITION, (float)(readLeftEncoder() - offsetTicks)});
    }
    if (getRightMotorControl().mode == POSITION) {
        setRightMotorControl({POSITION, getRightMotorControl().value + offsetTicks});
    } else {
        setRightMotorControl({POSITION, (float)(readRightEncoder() + offsetTicks)});
    }

    if (id != "NULL")
    {
        sendPacketOnPidComplete(id);
    }
}

// Stops the bot in its tracks
void stop() {
    setLeftPower(0);
    setRightPower(0);

    serialLogln("Bot Stopped!", 2);
}

boolean isRobotPidAtTarget() {
    if (!DO_PID)
        return true;

    boolean leftAtTarget, rightAtTarget;

    if (getLeftMotorControl().mode == POSITION)
    {
        leftAtTarget = approxEquals(getLeftMotorControl().value, profileA.currentPosition, PID_POSITION_TOLERANCE)
                    && approxEquals(profileA.currentVelocity, 0.0, PID_VELOCITY_TOLERANCE);

        serialLog("Left Position: ", 3);
        serialLog(profileA.currentPosition, 3);
        serialLog(", Target: ", 3);
        serialLog(getLeftMotorControl().value, 3);
        serialLog(", Velocity: ", 3);
        serialLog(profileA.currentVelocity, 3);
        serialLog(", At Target: ", 3);
        serialLogln(leftAtTarget, 3);
    }
    else
    {
        leftAtTarget = approxEquals(getLeftMotorControl().value, profileA.currentVelocity, PID_VELOCITY_TOLERANCE);
    }
    if (getRightMotorControl().mode == POSITION)
    {
        rightAtTarget = approxEquals(getRightMotorControl().value, profileB.currentPosition, PID_POSITION_TOLERANCE)
                     && approxEquals(profileB.currentVelocity, 0.0, PID_VELOCITY_TOLERANCE);

        serialLog("Right Position: ", 3);
        serialLog(profileB.currentPosition, 3);
        serialLog(", Target: ", 3);
        serialLog(getRightMotorControl().value, 3);
        serialLog(", Velocity: ", 3);
        serialLog(profileB.currentVelocity, 3);
        serialLog(", At Target: ", 3);
        serialLogln(rightAtTarget, 3);
    }
    else
    {
        rightAtTarget = approxEquals(getRightMotorControl().value, profileB.currentVelocity, PID_VELOCITY_TOLERANCE);
    }

    return leftAtTarget && rightAtTarget;
}

void sendPacketOnPidComplete(std::string id) {
    if (!DO_PID)
        createAndSendPacket(2, "fail", id);

    if (isRobotPidAtTarget()) {
        createAndSendPacket(2, "success", id);
    } else {
        // Run on next loop
        timerDelay(1, [id](){ sendPacketOnPidComplete(id); });
    }
}

// Reads in the light value of all light sensors
void readLight() {
    startLightReading();
}

// Test motor and encoder synchronization
void startMotorAndEncoderTest() {
    (new MotorEncoderTest())->startMotorAndEncoderTest();
}

// Tests the motors. This turns the motors on.
void startDriveTest() {
    drive(0.5f, 0.5f, "NULL");
    timerDelay(2000, &driveTestOff);
}

// Tests the motors. This turns the motors off.
void driveTestOff() {
    stop();
    timerDelay(2000, &startDriveTest);
}

#endif