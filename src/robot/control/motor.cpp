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

Motor::Motor(bool _inverted, int motor_pin_a, int motor_pin_b, uint8_t enc_pin_a, uint8_t enc_pin_b)
    : inverted(_inverted), encoder(enc_pin_a, enc_pin_b)
    {
    pin_a = motor_pin_a;
    pin_b = motor_pin_b;
    
    setupPWM(pin_a);
    setupPWM(pin_b);
}

void Motor::tick() {
    // Read from encoder and save its previous value
    prev_raw_enc_value = raw_enc_value;
    raw_enc_value = raw_dist();
}

// Returns the current power of the motor
float Motor::power() {
    if (inverted) {
        return -_power;
    }
    
    return _power;
}

// This will set how fast and what direction left motor will spin
// Value between [-1, 1]
// Negative is backwards, Positive is forwards
void Motor::power(float __power) {
    if (inverted) {
        __power = -__power;    
    }

    _power = __power;

    if (abs(_power) < MIN_MOTOR_POWER) {
        _power = 0;
    }

    
    if (_power > 0) {
        writePWM(pin_b, 0);
        writePWM(pin_a, mapPowerToDuty(_power));
    } else {
        writePWM(pin_a, 0);
        writePWM(pin_b, mapPowerToDuty(-_power));
    }

    serialLog("Motor Power: ", 4);
    serialLogln(_power, 4);
}

void Motor::encoder_reset() {
    encoder.readAndReset();
}

int32_t Motor::raw_dist() {
    if (inverted) {
        return -encoder.read();
    }
    
    return encoder.read();
}

double Motor::dist() {
    return ((double)raw_enc_value / TICKS_PER_ROTATION) * TIRE_CIRCUMFERENCE;
}

double Motor::tick_dist() {
    int32_t dist = raw_enc_value - prev_raw_enc_value;
    return ((double)dist / TICKS_PER_ROTATION) * TIRE_CIRCUMFERENCE;
}

#endif