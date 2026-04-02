#ifndef CHESSBOT_MOTOR_H
#define CHESSBOT_MOTOR_H

#include <stdint.h>

#include "utils/config.h"
#include "Encoder.h"

const float TIRE_RADIUS = 5.9;
const float TIRE_CIRCUMFERENCE = M_PI * 2 * TIRE_RADIUS;

class Motor {
    public:
        Motor(bool inverted, int motor_pin_a, int motor_pin_b, uint8_t enc_pin_a, uint8_t enc_pin_b);

        void tick();
        
        // Sets the motor power
        // power is a float between [-1, 1]
        float power();
        void power(float power);
        void encoder_reset();

        double dist(); // Total distance in cm
        double tick_dist(); // Distance this tick in cm
        int32_t raw_dist(); // Read the raw encoder value in ticks
    private:
        int pin_a;
        int pin_b;

        bool inverted;

        float _power;
        
        int32_t raw_enc_value;
        int32_t prev_raw_enc_value;
        Encoder encoder;


        
};

// Encoder EncoderA(ENCODER_A_PIN1, ENCODER_A_PIN2);
// void setupMotors();
// void setLeftPower(float power);
// void setRightPower(float power);

#endif