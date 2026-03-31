#ifndef CHESSBOT_LIGHT_SENSOR_H
#define CHESSBOT_LIGHT_SENSOR_H

// Built-In Libraries
#include "Arduino.h"

class Light {
    public:
        void tick();
        short value();
        unsigned long last_changed_time();

    private:
        int pin;
        short _value;
        unsigned long _last_changed_time;
};

void setupIR();
void activateIR();
void deactivateIR();

#endif