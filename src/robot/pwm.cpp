#ifndef CHESSBOT_PWM_CPP
#define CHESSBOT_PWM_CPP

// Associated Header File
#include "robot/pwm.h"

// Built-In Libraries
#include "Arduino.h"

// Custom Libraries
#include "utils/logging.h"
#include "utils/config.h"
#include "utils/functions.h"

// Sets a pin to be able to output
void setupPWM(int pin) {
    pinMode(pin, OUTPUT);
}

// Writes PWM to a pin. The duty cycle is a number
// between 0 and 255 that determines how often the signal is HIGH.
// You can refer to this link for more information on PWM
// https://learn.sparkfun.com/tutorials/pulse-width-modulation/all
void writePWM(int pin, int dutyCycle) {
    analogWrite(pin, dutyCycle);
}

// Takes in a power between [0, 1]
// We use this to change a float power between 0-1 to an int duty cycle between 0-4096
int mapPowerToDuty(float power) {
    int value = fmap(power, 0.0, 1.0, 0.0, 4096);
    serialLog((char*)"Mapped Duty: ", 4);
    serialLogln(value, 4);
    return value;
}

#endif