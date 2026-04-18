#include <Arduino.h>

#include "robot/motor.h"

#include "utils/config.h"
#include "utils/functions.h"
#include "utils/logging.h"

Motor::Motor(bool _inverted, int motor_pin_a, int motor_pin_b, uint8_t enc_pin_a, uint8_t enc_pin_b)
    : inverted(_inverted), encoder(enc_pin_a, enc_pin_b)
    {
    pin_a = motor_pin_a;
    pin_b = motor_pin_b;
    
    pinMode(pin_a, OUTPUT);
    pinMode(pin_b, OUTPUT);
}

void Motor::tick() {
    // Read from encoder and save its previous value
    prev_raw_enc_value = raw_enc_value;
    raw_enc_value = raw_dist();
}

int Motor::duty() {
    return power_to_duty(_power);
}

// Returns the current power of the motor
double Motor::power() {
    if (inverted) {
        return -_power;
    }
    
    return _power;
}

// This will set how fast and what direction left motor will spin
// Value between [-1, 1]
// Negative is backwards, Positive is forwards
void Motor::power(double __power) {
    if (inverted) {
        __power = -__power;    
    }

    _power = __power;
    
    analogWriteResolution(12);
    if (_power > 0) {
        analogWrite(pin_a, power_to_duty(_power));
        analogWrite(pin_b, 0);
    } else {
        analogWrite(pin_a, 0);
        analogWrite(pin_b, power_to_duty(_power));
    }
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

double Motor::get_saved() {
    return saved_dist;
}

double Motor::save_dist() {
    double old = saved_dist;
    saved_dist = dist();

    return old;
}

// Takes in a power between [0, 1]
// We use this to change a double power between 0-1 to an int duty cycle between 0-4096
int power_to_duty(double power) {
    power = abs(power);
    return fmap(power, 0.0, 1.0, 0.0, 4096.0);
}