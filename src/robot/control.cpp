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
    void setupBot() {
        setupMotors();
        setupIR();
        activateIR();
    }

    void drive(float tiles) {

    }

    void drive(float leftPower, float rightPower) {
        log((char*)"Drive: ");
        log(leftPower);
        log((char*)", ");
        logln(rightPower);
        setLeftPower(leftPower);
        setRightPower(rightPower);
    }

    void stop() {
        logln((char*)"Stop!");
        setLeftPower(0);
        setRightPower(0);
    }

    void readLight(int lightArray[]) {
        readLightLevels(lightArray);
    }

    void logLight() {
        logLightLevels();
    }

    void startDriveTest() {
        drive(0.5f, 0.5f);
        timerDelay(2000, &driveTestOff);
    }

    void driveTestOff() {
        stop();
        timerDelay(2000, &startDriveTest);
    }
};

#endif