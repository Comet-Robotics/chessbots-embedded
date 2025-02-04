#ifndef CHESSBOT_LIGHT_SENSOR_H
#define CHESSBOT_LIGHT_SENSOR_H

#include "Arduino.h"

namespace ChessBot
{
    void setupIR();
    void activateIR();
    void deactivateIR();
    void readLightLevels(int lightArray[]);
    void logLightLevels();
};

#endif