#ifndef CHESSBOT_MOTOR_CPP
#define CHESSBOT_MOTOR_CPP

// Associated Header File
#include "robot/motor.h"

// Built-In Libraries
#include "Arduino.h"

// Custom Libraries
#include "utils/logging.h"
#include "utils/config.h"
#include "robot/pwm.h"

// Sets up all the pins for the motors
void setupMotors() {
    serialLogln((char*)"Setting Up Motors...", 2);
    setupPWM(MOTOR_A_PIN1, 1);
    setupPWM(MOTOR_A_PIN2, 2);
    setupPWM(MOTOR_B_PIN1, 3);
    setupPWM(MOTOR_B_PIN2, 4);
    serialLogln((char*)"Motors Setup!", 2);
}

// This will set how fast and what direction left motor will spin
// Value between [-1, 1]
// Negative is backwards, Positive is forwards
void setLeftPower(float power) {
    // To spin the motor, you set one of the pins to 0, and the other to a PWM for speed.
    // The pins you set to each determine the spin direction
    if (power > 0) {
        writePWM(2, 0);
        writePWM(1, mapPowerToDuty(power));
    } else {
        writePWM(1, 0);
        writePWM(2, mapPowerToDuty(-power));
    }

    // Logs the power for debugging purposes
    serialLog((char*)"Left Power: ", 3);
    serialLogln(power, 3);
}

// This will set how fast and what direction right motor will spin
// Value between [-1, 1]
// Negative is backwards, Positive is forwards
void setRightPower(float power) {
    // To spin the motor, you set one of the pins to 0, and the other to a PWM for speed.
    // The pins you set to each determine the spin direction
    if (power > 0) {
        writePWM(4, 0);
        writePWM(3, mapPowerToDuty(power));
    } else {
        writePWM(3, 0);
        writePWM(4, mapPowerToDuty(-power));
    }

    // Logs the power for debugging purposes
    serialLog((char*)"Right Power: ", 3);
    serialLogln(power, 3);
}

#endif