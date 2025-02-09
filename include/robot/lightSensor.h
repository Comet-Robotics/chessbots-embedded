#ifndef CHESSBOT_LIGHT_SENSOR_H
#define CHESSBOT_LIGHT_SENSOR_H

// Built-In Libraries
#include "Arduino.h"

void setupIR();
void activateIR();
void deactivateIR();
void startLightReading();
void readLightLevels();

#endif