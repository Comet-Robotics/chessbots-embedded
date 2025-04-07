#ifndef CHESSBOT_LIGHT_SENSOR_H
#define CHESSBOT_LIGHT_SENSOR_H

// Built-In Libraries
#include "Arduino.h"

void setupIR();
void activateIR();
void deactivateIR();
void startLightReading(uint8_t counter, float prevTickVals[], const uint8_t TICK_DIST);
void readLightLevels();

#endif