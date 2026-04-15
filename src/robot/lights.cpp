#include <Arduino.h>

#include "robot/lights.h"

#include "../../env.h"
#include "utils/config.h"
#include "utils/logging.h"

static short LIGHT_RAW_VALUE_CUTOFF = 5000;
bool is_light_value_on(short value) {
    return value > LIGHT_RAW_VALUE_CUTOFF;
}

Light::Light(gpio_num_t _pin) {
    pin = _pin;
}

void Light::tick() {
    bool previous_value = _value;
    _changed_this_tick = false;

    _raw_value = analogRead(pin);
    
    _value = is_light_value_on(_raw_value);
    _held_value = _value || _held_value;
    
    if (_value != previous_value) {
        _changed_this_tick = true;
        _last_changed_time = millis();
    }
}

short Light::raw_value() {
    return _raw_value;
}

bool Light::value() {
    return is_light_value_on(_raw_value);
}

bool Light::held_value() {
    return _held_value;
}

void Light::reset() {
    _held_value = false;
}

bool Light::changed_this_tick() {
    return _changed_this_tick;
}

unsigned long Light::last_changed_time() {
    return _last_changed_time;
}

bool IR_activated = false;

// Sets the IR (Infrared) Blaster to be able to output
void setupIR() {
    serial_printf(DebugLevel::DEBUG, "Setting Up Light Sensors...\n");
    pinMode(RELAY_IR_LED_PIN, OUTPUT);
    serial_printf(DebugLevel::DEBUG, "Light Sensors Setup!\n");
}

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
    // if (!IR_activated) {
    //     return;
    // }

    // digitalWrite(RELAY_IR_LED_PIN, LOW);
    // IR_activated = false;
}