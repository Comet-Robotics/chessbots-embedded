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
    analogWriteResolution(12);
    serialLogln("Setting Up Motors...", 2);
    setupPWM(MOTOR_A_PIN1);
    setupPWM(MOTOR_A_PIN2);
    setupPWM(MOTOR_B_PIN1);
    setupPWM(MOTOR_B_PIN2);
    serialLogln("Motors Setup!", 2);
}

// This will set how fast and what direction left motor will spin
// Value between [-1, 1]
// Negative is backwards, Positive is forwards
void setLeftPower(float power) {
    // To spin the motor, you set one of the pins to 0, and the other to a PWM for speed.
    // The pins you set to each determine the spin direction
    if (power > 0) {
        writePWM(MOTOR_A_PIN2, 0);
        writePWM(MOTOR_A_PIN1, mapPowerToDuty(power));
    } else {
        writePWM(MOTOR_A_PIN1, 0);
        writePWM(MOTOR_A_PIN2, mapPowerToDuty(-power));
    }

    // Logs the power for debugging purposes
    serialLog("Left Power: ", 4);
    serialLogln(power, 4);
}

// This will set how fast and what direction right motor will spin
// Value between [-1, 1]
// Negative is backwards, Positive is forwards
void setRightPower(float power) {
    // To spin the motor, you set one of the pins to 0, and the other to a PWM for speed.
    // The pins you set to each determine the spin direction

    // Invert power so sending (1,1) to drive results in robot moving forward
    power *= -1;
    if (power > 0) {
        writePWM(MOTOR_B_PIN2, 0);
        writePWM(MOTOR_B_PIN1, mapPowerToDuty(power));
    } else {
        writePWM(MOTOR_B_PIN1, 0);
        writePWM(MOTOR_B_PIN2, mapPowerToDuty(-power));
    }

    // Logs the power for debugging purposes
    serialLog("Right Power: ", 4);
    serialLogln(power, 4);
}

#endif