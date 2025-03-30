#ifndef CHESSBOT_CONTROL_CPP
#define CHESSBOT_CONTROL_CPP

// Associated Header File
#include "robot/control.h"

// Built-In Libraries
#include "Arduino.h"

// Custom Libraries
#include "utils/logging.h"
#include "utils/timer.h"
#include "utils/config.h"
#include "robot/motor.h"
#include "robot/lightSensor.h"
#include "robot/encoder_new.h"
#include "robot/pid_controller.h"
#include "robot/driveTest.h"
#include "../../env.h"
#include <algorithm>

PIDController encoderAController = PIDController(1, 0, 0, -20000, +20000);
PIDController encoderBController = PIDController(1, 0, 0, -20000, +20000);

PIDController encoderAVelocityController(0.00006009999, 0.00038, 0, -1, +1);
PIDController encoderBVelocityController(0.00006009999, 0.00038, 0, -1, +1);

int encoderATarget = 0;
int encoderBTarget = 0;
int prevPositionA = 0;
int prevPositionB = 0;
boolean testEncoderPID_value = false;
void testEncoderPID()
{
    serialLogln((char *)"Changing encoder PID setpoint!", 2);
    if (testEncoderPID_value)
    {
        testEncoderPID_value = false;
        encoderATarget = 11900*3;
        encoderBTarget = -11900*3;
    }
    else
    {
        testEncoderPID_value = true;
        encoderATarget = encoderBTarget = 0;
    }
}

// Sets up all the aspects needed for the bot to work
void setupBot() {
    serialLogln((char*)"Setting Up Bot...", 2);
    setupMotors();
    setupIR();
    setupEncodersNew();
    serialLogln((char*)"Bot Set Up!", 2);

    encoderAController.Reset();
    encoderBController.Reset();

    if (DO_PID_TEST)
        timerInterval(15000, &testEncoderPID);
}

// Manages control loop (loopDelayMs is for reference)
void controlLoop(int loopDelayMs) {
    if (DO_LIGHT_SENSOR_TEST)
        readLight();

    if (DO_ENCODER_TEST)
        encoderLoop();

    if (DO_PID_TEST) {
        double loopDelaySeconds = ((double) loopDelayMs) / 1000;

        double currentPositionEncoderA = readEncoderA();
        double currentPositionEncoderB = readEncoderB();

        double desiredVelocityA = encoderAController.Compute(encoderATarget, currentPositionEncoderA, loopDelaySeconds);
        double desiredVelocityB = encoderBController.Compute(encoderBTarget, currentPositionEncoderB, loopDelaySeconds);

        double currentVelocityA = (currentPositionEncoderA - prevPositionA) / loopDelaySeconds;
        double currentVelocityB = (currentPositionEncoderB - prevPositionB) / loopDelaySeconds;

        prevPositionA = currentPositionEncoderA;
        prevPositionB = currentPositionEncoderB;

        double leftMotorPower = encoderAVelocityController.Compute(desiredVelocityA, currentVelocityA, loopDelaySeconds);
        double rightMotorPower = encoderBVelocityController.Compute(desiredVelocityB, currentVelocityB, loopDelaySeconds);

        serialLog((char *)"Encoder A ", 2);
        serialLog((float) currentPositionEncoderA, 2);
        serialLog((char *)" Encoder B ", 2);
        serialLog((float) currentPositionEncoderB, 2);
        serialLog((char *)" Desired Velocity A ", 2);
        serialLog((float) desiredVelocityA, 2);
        serialLog((char *)" Desired Velocity B ", 2);
        serialLog((float) desiredVelocityB, 2);
        serialLog((char *)" Current Velocity A ", 2);
        serialLog((float) currentVelocityA, 2);
        serialLog((char *)" Current Velocity B ", 2);
        serialLog((float) currentVelocityB, 2);
        serialLog((char *)" Left Motor Power ", 2);
        serialLog((float) leftMotorPower, 2);
        serialLog((char *)" Right Motor Power ", 2);
        serialLogln((float) rightMotorPower, 2);
        
        drive(
            leftMotorPower,
            rightMotorPower
        );
    }
}

// Drives a specific amount of tiles (WIP)
void drive(float tiles) {

}

// Drives the wheels according to the powers set. Negative is backwards, Positive forwards
void drive(float leftPower, float rightPower) {
    // TODO: maybe move to motor.cpp?
    float minPower = 0.14;
    if (leftPower < minPower && leftPower > -minPower)
    {
        leftPower = 0;
    }
    if (rightPower < minPower && rightPower > -minPower)
    {
        rightPower = 0;
    }

    setLeftPower(leftPower);
    setRightPower(rightPower);

    // Logs the Drive values for debugging purposes
    serialLog((char*)"Drive: ", 3);
    serialLog(leftPower, 3);
    serialLog((char*)", ", 3);
    serialLogln(rightPower, 3);
}

// Stops the bot in its tracks
void stop() {
    setLeftPower(0);
    setRightPower(0);

    serialLogln((char*)"Bot Stopped!", 2);
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
    drive(0.5f, 0.5f);
    timerDelay(2000, &driveTestOff);
}

// Tests the motors. This turns the motors off.
void driveTestOff() {
    stop();
    timerDelay(2000, &startDriveTest);
}

#endif