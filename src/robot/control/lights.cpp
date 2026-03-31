#ifndef CHESSBOT_LIGHT_SENSOR_CPP
#define CHESSBOT_LIGHT_SENSOR_CPP

// Associated Header File
#include "robot/control/lights.h"
#include "../../../env.h"

// Built-In Libraries
#include "Arduino.h"

// Custom Libraries
#include "utils/logging.h"
#include "utils/config.h"

void Light::tick() {
    short prev = _value;

    _value = analogRead(pin);

    if (prev != _value) {
        _last_changed_time = millis();
    }
}

short Light::value() {
    return _value;
}

unsigned long Light::last_changed_time() {
    return _last_changed_time;
}

bool IR_activated = false;

// Sets the IR (Infrared) Blaster to be able to output
void setupIR() {
    serialLogln("Setting Up Light Sensors...", 2);
    pinMode(RELAY_IR_LED_PIN, OUTPUT);
    serialLogln("Light Sensors Setup!", 2);
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
    if (!IR_activated) {
        return;
    }

    digitalWrite(RELAY_IR_LED_PIN, LOW);
    IR_activated = false;
}

#endif