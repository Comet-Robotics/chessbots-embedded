#ifndef CHESSBOT_BATTERY_CPP
#define CHESSBOT_BATTERY_CPP

// Associated Header File
#include "robot/battery.h"

// Built-In Libraries
#include "Arduino.h"

// Custom Libraries
#include "utils/config.h"

int batteryVoltageOffset = 100;

int getBatteryLevel() {
    return analogRead(BATTERY_VOLTAGE_PIN) - batteryVoltageOffset;
}

#endif