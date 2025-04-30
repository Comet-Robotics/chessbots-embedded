#ifndef CHESSBOT_LIGHT_SENSOR_H
#define CHESSBOT_LIGHT_SENSOR_H

// Built-In Libraries
#include "Arduino.h"

void setupIR();
void readGarbageVals();
void activateIR();
void deactivateIR();
void startLightReading(bool* onFirstTile, bool* waitingForLight);
void readLightLevels();

#endif