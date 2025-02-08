#ifndef CHESSBOT_CONTROL_CPP
#define CHESSBOT_CONTROL_CPP

#include "robot/control.h"

#include "Arduino.h"

#include "utils/logging.h"
#include "utils/timer.h"
#include "robot/motor.h"
#include "robot/lightSensor.h"

namespace ChessBot
{
    // Sets up all the aspects needed for the bot to work
    void setupBot() {
        setupMotors();
        setupIR();
    }

    // Drives a specific amount of tiles (WIP)
    void drive(float tiles) {

    }

    // Drives the wheels according to the powers set. Negative is backwards, Positive forwards
    void drive(float leftPower, float rightPower) {
        setLeftPower(leftPower);
        setRightPower(rightPower);

        // Logs the Drive values for debugging purposes
        log((char*)"Drive: ", 2);
        log(leftPower, 2);
        log((char*)", ", 2);
        logln(rightPower, 2);
    }

    // Stops the bot in its tracks
    void stop() {
        setLeftPower(0);
        setRightPower(0);

        logln((char*)"Bot Stopped!", 2);
    }

    // Reads in the light value of all light sensors
    void readLight() {
        startLightReading();
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
};

#endif