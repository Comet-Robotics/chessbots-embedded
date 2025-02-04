#ifndef CHESSBOT_MOTOR_CPP
#define CHESSBOT_MOTOR_CPP

#include "robot/motor.h"

#include "Arduino.h"
#include "utils/logging.h"
#include "utils/config.h"
#include "robot/pwm.h"

namespace ChessBot
{
    void setupMotors() {
        setupPWM(MOTOR_A_PIN1, 1);
        setupPWM(MOTOR_A_PIN2, 2);
        setupPWM(MOTOR_B_PIN1, 3);
        setupPWM(MOTOR_B_PIN2, 4);
    }

    // Value between [-1, 1]
    void setLeftPower(float power) {
        log((char*)"Left Power: ");
        logln(power);
        if (power > 0) {
            writePWM(2, 0);
            writePWM(1, mapPowerToDuty(power));
        } else {
            writePWM(1, 0);
            writePWM(2, mapPowerToDuty(-power));
        }
    }

    // Value between [-1, 1]
    void setRightPower(float power) {
        log((char*)"Right Power: ");
        logln(power);
        if (power > 0) {
            writePWM(4, 0);
            writePWM(3, mapPowerToDuty(power));
        } else {
            writePWM(3, 0);
            writePWM(4, mapPowerToDuty(-power));
        }
    }
};

#endif