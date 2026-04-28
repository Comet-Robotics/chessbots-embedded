#pragma once

#include <Encoder.h>
#include <stdint.h>

#include "robot/pid.h"
#include "utils/functions.h"
#include "utils/config.h"


const double TIRE_RADIUS = 5.9;
const double TIRE_CIRCUMFERENCE = M_PI * 2 * TIRE_RADIUS;

class Motor {
    public:
        Motor(bool inverted, int motor_pin_a, int motor_pin_b, uint8_t enc_pin_a, uint8_t enc_pin_b);

        void tick(uint32_t delay);

        /* Read-only stats */ 

        int duty();
        double power();
        double speed();
        double target_speed();
        double dist();      // Total distance in cm
        double tick_dist(); // Distance this tick in cm
        int32_t raw_dist(); // Read the raw encoder value in ticks
        double get_saved();

        /* Affect the motor */

        double save_dist();
        void encoder_reset();
        void set_power(double power);
        void set_target_speed(double speed);

    private:
        int pin_a;
        int pin_b;

        bool inverted;

        PIDController speed_controller;
        double _power;
        RingBuf _speeds;
        double _target_speed;

        int32_t raw_enc_value;
        int32_t prev_raw_enc_value;
        double saved_dist;
        Encoder encoder;
};

int power_to_duty(double power);