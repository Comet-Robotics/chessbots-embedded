#pragma once

#include <Encoder.h>
#include <stdint.h>

#include "utils/config.h"


const double TIRE_RADIUS = 5.9;
const double TIRE_CIRCUMFERENCE = M_PI * 2 * TIRE_RADIUS;

class Motor {
    public:
        Motor(bool inverted, int motor_pin_a, int motor_pin_b, uint8_t enc_pin_a, uint8_t enc_pin_b);

        void tick();
        
        // Sets the motor power
        // power is a double between [-1, 1]
        double power();
        void power(double power);
        void encoder_reset();

        double dist(); // Total distance in cm
        double tick_dist(); // Distance this tick in cm
        int32_t raw_dist(); // Read the raw encoder value in ticks
    private:
        int pin_a;
        int pin_b;

        bool inverted;

        double _power;
        
        int32_t raw_enc_value;
        int32_t prev_raw_enc_value;
        Encoder encoder;
};

int power_to_duty(double power);