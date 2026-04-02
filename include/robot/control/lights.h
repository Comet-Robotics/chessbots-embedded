#ifndef CHESSBOT_LIGHT_SENSOR_H
#define CHESSBOT_LIGHT_SENSOR_H

// Built-In Libraries
#include "Arduino.h"

class Light {
    public:
        Light(gpio_num_t pin);

        void tick();
        
        short raw_value();
        bool value();
        bool held_value();
        void reset();

        bool changed_this_tick();
        unsigned long last_changed_time();

    private:
        gpio_num_t pin;
        short _raw_value;
        bool _held_value;
        bool _changed_this_tick;
        unsigned long _last_changed_time;
};

void setupIR();
void activateIR();
void deactivateIR();

#endif