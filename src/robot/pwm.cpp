#ifndef CHESSBOT_PWM_CPP
#define CHESSBOT_PWM_CPP

#include "robot/pwm.h"

#include "Arduino.h"
#include "utils/logging.h"
#include "utils/config.h"

#include "driver/ledc.h"

namespace ChessBot
{
    constexpr static uint8_t LEDC_DUTY_RES = 14;
    static uint16_t LEDC_DUTY_STEPS = (1 << LEDC_DUTY_RES) - 1;
    constexpr static int16_t LEDC_FREQUENCY = 5000;

    void setupPWM(int pin, int channel) {
        pinMode(pin, OUTPUT);
        ledcSetup(channel, LEDC_FREQUENCY, LEDC_DUTY_RES);
        ledcAttachPin(pin, channel);
    }

    void writePWM(int channel, int dutyCycle) {
        ledcWrite(channel, dutyCycle);
    }

    float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    // Value between [0, 1]
    int mapPowerToDuty(float power) {
        int value = fmap(power, 0.0, 1.0, 0.0, LEDC_DUTY_STEPS);
        log((char*)"Mapped Duty: ");
        logln(value);
        return value;
    }
};

#endif