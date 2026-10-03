#include <Arduino.h>

#include "robot/ir.h"

#include "../../env.h"
#include "utils/config.h"
#include "utils/logging.h"

static short LIGHT_RAW_VALUE_CUTOFF = 5000;
bool is_ir_value_high(short value) {
    return value < LIGHT_RAW_VALUE_CUTOFF;
}

IRSensor::IRSensor(gpio_num_t _pin) {
    pin = _pin;
}

void IRSensor::tick() {
    bool previous_value = _value;
    _changed_this_tick = false;

    _raw_value = analogRead(pin);
    
    _value = is_ir_value_high(_raw_value);
    _held_value = _value || _held_value;
    
    if (_value != previous_value) {
        _changed_this_tick = true;
        _last_changed_time = millis();
    }
}

short IRSensor::raw_value() {
    return _raw_value;
}

bool IRSensor::value() {
    return is_ir_value_high(_raw_value);
}

bool IRSensor::held_value() {
    return _held_value;
}

void IRSensor::reset() {
    _held_value = false;
}

bool IRSensor::changed_this_tick() {
    return _changed_this_tick;
}

unsigned long IRSensor::last_changed_time() {
    return _last_changed_time;
}

bool IR_activated = false;

// Turns on the IR Blaster
// Does turning on and off the IR potentially take more energy than just leaving it on?
void activateIR() {
    if (IR_activated) {
        return;
    }

    digitalWrite(RELAY_IR_LED_PIN, HIGH);
    IR_activated = true;
}

// Turns off the IR Blaster
void deactivateIR() {
    if (!IR_activated) {
        return;
    }

    digitalWrite(RELAY_IR_LED_PIN, LOW);
    IR_activated = false;
}