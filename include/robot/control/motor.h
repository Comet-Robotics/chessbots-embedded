#ifndef CHESSBOT_MOTOR_H
#define CHESSBOT_MOTOR_H

#include "config.h"
#include "Encoder.h"

//just measured, its 5.9 centimeters, or .059 meters
const float TIRE_RADIUS = 0.059;
//circumference equals pi * diameter. In meters
const float TIRE_CIRCUMFERENCE = M_PI * 2 * TIRE_RADIUS;

class Motor {
    public:
        Motor(int pin_a, int pin_b);

        void tick();
        
        // Sets the motor power
        // power is a float between [-1, 1]
        void power(float power);
        void encoder_reset();

        // The distance of the tire in cm
        float dist();
    private:
        int pin_a;
        int pin_b;

        Encoder encoder;

        // The raw encoder value
        float dist_raw();
        
};

// Encoder EncoderA(ENCODER_A_PIN1, ENCODER_A_PIN2);
// void setupMotors();
// void setLeftPower(float power);
// void setRightPower(float power);

#endif