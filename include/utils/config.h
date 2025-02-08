#ifndef CHESSBOT_CONFIG_H
#define CHESSBOT_CONFIG_H

#include "Arduino.h"

namespace ChessBot
{
    static gpio_num_t MOTOR_A_PIN1 = GPIO_NUM_33;
    static gpio_num_t MOTOR_A_PIN2 = GPIO_NUM_38;
    static gpio_num_t MOTOR_B_PIN1 = GPIO_NUM_39;
    static gpio_num_t MOTOR_B_PIN2 = GPIO_NUM_40;

    static gpio_num_t ENCODER_A_PIN1 = GPIO_NUM_32;
    static gpio_num_t ENCODER_A_PIN2 = GPIO_NUM_31;
    static gpio_num_t ENCODER_B_PIN1 = GPIO_NUM_18;
    static gpio_num_t ENCODER_B_PIN2 = GPIO_NUM_34;

    static gpio_num_t RELAY_IR_LED_PIN = GPIO_NUM_15;
    static gpio_num_t PHOTODIODE_A_PIN = GPIO_NUM_1;
    static gpio_num_t PHOTODIODE_B_PIN = GPIO_NUM_2;
    static gpio_num_t PHOTODIODE_C_PIN = GPIO_NUM_4;
    static gpio_num_t PHOTODIODE_D_PIN = GPIO_NUM_6;

    static float_t WHEEL_DIAMETER_INCHES = 4.375;

    static float_t MOTOR_A_DRIVE_MULTIPLIER;
    static float_t MOTOR_B_DRIVE_MULTIPLIER;

    static float_t ENCODER_MULTIPLIER;
};

#endif