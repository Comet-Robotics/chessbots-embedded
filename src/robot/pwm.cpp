#ifndef CHESSBOT_PWM_CPP
#define CHESSBOT_PWM_CPP

#include "robot/pwm.h"

#include "Arduino.h"
#include "utils/logging.h"
#include "utils/config.h"

#include "driver/ledc.h"

namespace ChessBot
{
    void setupPWM(int pin) {
        pinMode(pin, OUTPUT);
    }

    void writePWM(int pin, int dutyCycle) {
        analogWrite(pin, dutyCycle);
    }

    float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    // Value between [0, 1]
    int mapPowerToDuty(float power) {
        int value = fmap(power, 0.0, 1.0, 0.0, 255);
        log((char*)"Mapped Duty: ");
        logln(value);
        return value;
    }
};

#endif