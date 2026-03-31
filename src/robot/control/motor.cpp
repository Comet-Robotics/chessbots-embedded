#ifndef CHESSBOT_MOTOR_CPP
#define CHESSBOT_MOTOR_CPP

// Associated Header File
#include "robot/control/motor.h"

// Built-In Libraries
#include "Arduino.h"

// Custom Libraries
#include "robot/pwm.h"
#include "utils/config.h"
#include "utils/logging.h"

Motor::Motor(int motor_pin_a, int motor_pin_b, int enc_pin_a, int enc_pin_b) {
    pin_a = _pin_a;
    pin_b = _pin_b;
    
    setupPWM(pin_a);
    setupPWM(pin_b);

    encoder(enc_pin_a, enc_pin_b);
}

void Motor::tick() {
    // Read from encoder and save its previous value
}

// This will set how fast and what direction left motor will spin
// Value between [-1, 1]
// Negative is backwards, Positive is forwards
void Motor::power(float power) {
    if (power > 0) {
        writePWM(MOTOR_A_PIN2, 0);
        writePWM(MOTOR_A_PIN1, mapPowerToDuty(power));
    } else {
        writePWM(MOTOR_A_PIN1, 0);
        writePWM(MOTOR_A_PIN2, mapPowerToDuty(-power));
    }

    serialLog("Motor Power: ", 4);
    serialLogln(power, 4);
}

void Motor::encoder_reset() {
    encoder.readAndReset();
}

float Motor::dist_raw() {
    return encoder.read();
}

float Motor::dist() {
    return dist_raw() * TIRE_CIRCUMFERENCE;
}

#endif