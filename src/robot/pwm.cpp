#ifndef CHESSBOT_PWM_CPP
#define CHESSBOT_PWM_CPP

// Associated Header File
#include "robot/pwm.h"

// Built-In Libraries
#include "Arduino.h"
#include "driver/ledc.h"

// Custom Libraries
#include "utils/logging.h"
#include "utils/config.h"
#include "utils/functions.h"

constexpr static uint8_t LEDC_DUTY_RES = 13;
static uint16_t LEDC_DUTY_STEPS = (1 << LEDC_DUTY_RES) - 1;
constexpr static int16_t LEDC_FREQUENCY = 5000;

// Sets a pin to be able to output
void setupPWM(int pin, int channel) {
    pinMode(pin, OUTPUT);
    ledcSetup(channel, LEDC_FREQUENCY, LEDC_DUTY_RES);
    ledcAttachPin(pin, channel);
}

// Writes PWM to a pin. The duty cycle is a number
// between 0 and 255 that determines how often the signal is HIGH.
// You can refer to this link for more information on PWM
// https://learn.sparkfun.com/tutorials/pulse-width-modulation/all
void writePWM(int channel, int dutyCycle) {
    ledcWrite(channel, dutyCycle);
}

// Takes in a power between [0, 1]
// We use this to change a float power between 0-1 to an int duty cycle between 0-2^LEDC_DUTY_RES
int mapPowerToDuty(float power) {
    int value = fmap(power, 0.0, 1.0, 0.0, LEDC_DUTY_STEPS);
    serialLog((char*)"Mapped Duty: ", 4);
    serialLogln(value, 4);
    return value;
}

#endif