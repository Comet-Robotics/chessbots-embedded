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
#include "wifi/connection.h"
#include "robot/encoder.h"
#include "robot/pidController.h"
#include "robot/driveTest.h"
#include "../../env.h"
#include <algorithm>

PIDController encoderAController = PIDController(1, 0, 0, -20000, +20000);
PIDController encoderBController = PIDController(1, 0, 0, -20000, +20000);

PIDController encoderAVelocityController(0.00006, 0, 0, -1, +1);
PIDController encoderBVelocityController(0.00006, 0, 0, -1, +1);

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
        encoderATarget = encoderBTarget = TICKS_PER_ROTATION*10;
    }
    else
    {
        testEncoderPID_value = true;
        encoderATarget = encoderBTarget = 0;
    }
}

int testTurn_angle = 0;
void testTurn()
{
    serialLog((char *)"Changing destination angle to ", 2);
    serialLog(testTurn_angle, 2);
    testTurn_angle = (testTurn_angle + 90) % 360;
    turn(M_PI / 2, "NULL");
    serialLog((char *)" (", 2);
    serialLog(encoderATarget, 2);
    serialLog((char *)", ", 2);
    serialLog(encoderBTarget, 2);
    serialLogln((char *)")", 2);
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
        timerInterval(20000, &testEncoderPID);
    
    if (DO_TURN_TEST)
        timerInterval(5000, &testTurn);
}
// + (0.0001 * desiredVelocityA)
// Manages control loop (loopDelayMs is for reference)
void controlLoop(int loopDelayMs) {
    if (DO_LIGHT_SENSOR_TEST)
        readLight();

    if (DO_ENCODER_TEST)
        encoderLoop();

    if (DO_PID) {
        double loopDelaySeconds = ((double) loopDelayMs) / 1000;

        int currentPositionEncoderA = readLeftEncoder();
        int currentPositionEncoderB = readRightEncoder();

        double desiredVelocityA = encoderAController.Compute(encoderATarget, currentPositionEncoderA, loopDelaySeconds);
        double desiredVelocityB = encoderBController.Compute(encoderBTarget, currentPositionEncoderB, loopDelaySeconds);

        double currentVelocityA = (currentPositionEncoderA - prevPositionA) / loopDelaySeconds;
        double currentVelocityB = (currentPositionEncoderB - prevPositionB) / loopDelaySeconds;

        prevPositionA = currentPositionEncoderA;
        prevPositionB = currentPositionEncoderB;

        double leftMotorPower = (encoderAVelocityController.Compute(desiredVelocityA, currentVelocityA, loopDelaySeconds));
        double rightMotorPower = (encoderBVelocityController.Compute(desiredVelocityB, currentVelocityB, loopDelaySeconds));

        serialLog(currentPositionEncoderA, 3);
        serialLog((char *)",", 3);
        serialLog(currentPositionEncoderB, 3);
        serialLog((char *)",", 3);
        serialLog((float) desiredVelocityA, 3);
        serialLog((char *)",", 3);
        serialLog((float) desiredVelocityB, 3);
        serialLog((char *)",", 3);
        serialLog((float) currentVelocityA, 3);
        serialLog((char *)",", 3);
        serialLog((float) currentVelocityB, 3);
        serialLog((char *)",", 3);
        serialLog((float) leftMotorPower, 3);
        serialLog((char *)",", 3);
        serialLog((float) rightMotorPower, 3);
        serialLog((char *)",", 3);
        serialLog(encoderATarget, 3);
        serialLog((char *)",", 3);
        serialLogln(encoderBTarget, 3);

        drive(
            leftMotorPower,
            rightMotorPower,
            "NULL"
        );
    }
}

// Drives a specific amount of tiles (WIP)
void drive(float tiles) {

}

void driveTicks(int tickDistance, std::string id)
{
    encoderATarget = readLeftEncoder() + tickDistance;
    encoderBTarget = readRightEncoder() - tickDistance;

}

// Drives the wheels according to the powers set. Negative is backwards, Positive forwards
void drive(float leftPower, float rightPower, std::string id) {
    // TODO: maybe move to motor.cpp?
    float minPower = 0.16;
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

    //we only send null as id during our test drive. The only other time this drive method is called will be
    //when the server sends it, meaning it will have an id to send back.
    if(id != "NULL")
    {
        createAndSendPacket(2, "success", id);   
    }
}

void turn(float angleRadians, std::string id) {
    float offsetInches = TRACK_WIDTH_INCHES * angleRadians / 2;
    int offsetTicks = (int) (offsetInches / (WHEEL_DIAMETER_INCHES * M_PI) * TICKS_PER_ROTATION);

    encoderATarget -= offsetTicks;
    encoderBTarget += offsetTicks;

    // TODO wait for pid to reach target
    if (id != "NULL")
    {
        createAndSendPacket(2, "success", id);
    }
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
    drive(0.5f, 0.5f, "NULL");
    timerDelay(2000, &driveTestOff);
}

// Tests the motors. This turns the motors off.
void driveTestOff() {
    stop();
    timerDelay(2000, &startDriveTest);
}

#endif