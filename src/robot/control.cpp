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
        leftMotorControl = rightMotorControl = { POSITION, (float)TICKS_PER_ROTATION*5 };
    }
    else
    {
        testEncoderPID_value = false;
        leftMotorControl = rightMotorControl = { POSITION, 0.0f };
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
    serialLog(leftMotorControl.value, 2);
    serialLog(", ", 2);
    serialLog(rightMotorControl.value, 2);
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

        if (leftMotorControl.mode == POSITION) {
            profileA.targetPosition = leftMotorControl.value;
            desiredVelocityA = updateTrapezoidalProfile(profileA, loopDelaySeconds);
        } else {
            desiredVelocityA = leftMotorControl.value;
        }
        if (rightMotorControl.mode == POSITION) {
            profileB.targetPosition = rightMotorControl.value;
            desiredVelocityB = updateTrapezoidalProfile(profileB, loopDelaySeconds);
        } else {
            desiredVelocityB = rightMotorControl.value;
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

        serialLog(currentPositionEncoderA, 3);
        serialLog(",", 3);
        serialLog(currentPositionEncoderB, 3);
        serialLog(",", 3);
        serialLog((float) desiredVelocityA, 3);
        serialLog(",", 3);
        serialLog((float) desiredVelocityB, 3);
        serialLog(",", 3);
        serialLog((float) currentVelocityA, 3);
        serialLog(",", 3);
        serialLog((float) currentVelocityB, 3);
        serialLog(",", 3);
        serialLog((float) leftMotorPower, 3);
        serialLog(",", 3);
        serialLog((float) rightMotorPower, 3);
        serialLog(",", 3);
        serialLog(leftMotorControl.mode == POSITION ? leftMotorControl.value : 0, 3);
        serialLog(",", 3);
        serialLog(rightMotorControl.mode == POSITION ? rightMotorControl.value : 0, 3); // TODO log results of trapezoidal profile into csv (on motor value graph)
        serialLog(",", 3);
        serialLogln((float) loopDelaySeconds, 3);

        drive(
            leftMotorPower, // leftMotorPower,
            rightMotorPower, // rightMotorPower,
            "NULL"
        );
    }
}

// Drives a specific amount of tiles (WIP)
void drive(float tiles) {
    if (!getStoppedStatus()) {
    }
}

void driveTicks(int tickDistance, std::string id)
{
    if (!getStoppedStatus()) {
        leftMotorControl = { POSITION, (float)(readLeftEncoder() + tickDistance) };
        rightMotorControl = { POSITION, (float)(readRightEncoder() + tickDistance) };

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
    float offsetInches = TRACK_WIDTH_INCHES * angleRadians / 2;
    int offsetTicks = (int) (offsetInches / (WHEEL_DIAMETER_INCHES * M_PI) * TICKS_PER_ROTATION);

    if (leftMotorControl.mode == POSITION) {
        leftMotorControl.value -= offsetTicks;
    } else {
        leftMotorControl = { POSITION, (float)(readLeftEncoder() - offsetTicks) };
    }
    if (rightMotorControl.mode == POSITION) {
        rightMotorControl.value += offsetTicks;
    } else {
        rightMotorControl = { POSITION, (float)(readRightEncoder() + offsetTicks) };
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

    float POSITION_TOLERANCE = 100;
    float VELOCITY_TOLERANCE = 200;

    boolean leftAtTarget, rightAtTarget;

    if (leftMotorControl.mode == POSITION)
    {
        leftAtTarget = (leftMotorControl.value - POSITION_TOLERANCE) <= profileA.currentPosition && profileA.currentPosition <= (leftMotorControl.value + POSITION_TOLERANCE)
                    && (-VELOCITY_TOLERANCE) <= profileA.currentVelocity && profileA.currentVelocity <= VELOCITY_TOLERANCE;
    }
    else
    {
        leftAtTarget = (leftMotorControl.value - VELOCITY_TOLERANCE) <= profileA.currentVelocity && profileA.currentVelocity <= (leftMotorControl.value + VELOCITY_TOLERANCE);
    }
    if (rightMotorControl.mode == POSITION)
    {
        rightAtTarget = (rightMotorControl.value - POSITION_TOLERANCE) <= profileB.currentPosition && profileB.currentPosition <= (rightMotorControl.value + POSITION_TOLERANCE)
                     && (-VELOCITY_TOLERANCE) <= profileB.currentVelocity && profileB.currentVelocity <= VELOCITY_TOLERANCE;
    }
    else
    {
        rightAtTarget = (rightMotorControl.value - VELOCITY_TOLERANCE) <= profileB.currentVelocity && profileB.currentVelocity <= (rightMotorControl.value + VELOCITY_TOLERANCE);
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