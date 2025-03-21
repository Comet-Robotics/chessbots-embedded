#ifndef CHESSBOT_CONFIG_CPP
#define CHESSBOT_CONFIG_CPP

// Associated Header File
#include "utils/logging.h"
#include "utils/config.h"

// Built-In Libraries
#include "Arduino.h"

// External Libraries
#include <ArduinoJson.h>

gpio_num_t MOTOR_A_PIN1 = GPIO_NUM_33;
gpio_num_t MOTOR_A_PIN2 = GPIO_NUM_38;
gpio_num_t MOTOR_B_PIN1 = GPIO_NUM_39;
gpio_num_t MOTOR_B_PIN2 = GPIO_NUM_40;

gpio_num_t ENCODER_A_PIN1 = GPIO_NUM_18;
gpio_num_t ENCODER_A_PIN2 = GPIO_NUM_21;
gpio_num_t ENCODER_B_PIN1 = GPIO_NUM_16;
gpio_num_t ENCODER_B_PIN2 = GPIO_NUM_17;

gpio_num_t RELAY_IR_LED_PIN = GPIO_NUM_15;
gpio_num_t PHOTODIODE_A_PIN = GPIO_NUM_1;
gpio_num_t PHOTODIODE_B_PIN = GPIO_NUM_2;
gpio_num_t PHOTODIODE_C_PIN = GPIO_NUM_4;
gpio_num_t PHOTODIODE_D_PIN = GPIO_NUM_6;

float WHEEL_DIAMETER_INCHES = 4.375;

float MOTOR_A_DRIVE_MULTIPLIER = 1.0;
float MOTOR_B_DRIVE_MULTIPLIER = 1.0;

float ENCODER_MULTIPLIER = 1.0;

void setConfig(JsonObject config) {
    serialLogln((char*)"Setting Config...", 2);

    // The is<x>() method checks the type of the variable. If the type isn't none, then
    // we know this variable exists in the config and has a value
    if (config["MOTOR_A_PIN1"].is<gpio_num_t>()) MOTOR_A_PIN1 = config["MOTOR_A_PIN1"];
    if (config["MOTOR_A_PIN2"].is<gpio_num_t>()) MOTOR_A_PIN2 = config["MOTOR_A_PIN2"];
    if (config["MOTOR_B_PIN1"].is<gpio_num_t>()) MOTOR_B_PIN1 = config["MOTOR_B_PIN1"];
    if (config["MOTOR_B_PIN2"].is<gpio_num_t>()) MOTOR_B_PIN2 = config["MOTOR_B_PIN2"];

    if (config["ENCODER_A_PIN1"].is<gpio_num_t>()) ENCODER_A_PIN1 = config["ENCODER_A_PIN1"];
    if (config["ENCODER_A_PIN2"].is<gpio_num_t>()) ENCODER_A_PIN2 = config["ENCODER_A_PIN2"];
    if (config["ENCODER_B_PIN1"].is<gpio_num_t>()) ENCODER_B_PIN1 = config["ENCODER_B_PIN1"];
    if (config["ENCODER_B_PIN2"].is<gpio_num_t>()) ENCODER_B_PIN2 = config["ENCODER_B_PIN2"];

    if (config["RELAY_IR_LED_PIN"].is<gpio_num_t>()) RELAY_IR_LED_PIN = config["RELAY_IR_LED_PIN"];
    if (config["PHOTODIODE_B_PIN"].is<gpio_num_t>()) PHOTODIODE_B_PIN = config["PHOTODIODE_B_PIN"];
    if (config["PHOTODIODE_C_PIN"].is<gpio_num_t>()) PHOTODIODE_C_PIN = config["PHOTODIODE_C_PIN"];
    if (config["PHOTODIODE_D_PIN"].is<gpio_num_t>()) PHOTODIODE_D_PIN = config["PHOTODIODE_D_PIN"];
    if (config["PHOTODIODE_A_PIN"].is<gpio_num_t>()) PHOTODIODE_A_PIN = config["PHOTODIODE_A_PIN"];

    if (config["WHEEL_DIAMETER_INCHES"].is<float>()) WHEEL_DIAMETER_INCHES = config["WHEEL_DIAMETER_INCHES"];

    if (config["MOTOR_A_DRIVE_MULTIPLIER"].is<float>()) MOTOR_A_DRIVE_MULTIPLIER = config["MOTOR_A_DRIVE_MULTIPLIER"];
    if (config["MOTOR_B_DRIVE_MULTIPLIER"].is<float>()) MOTOR_B_DRIVE_MULTIPLIER = config["MOTOR_B_DRIVE_MULTIPLIER"];

    if (config["ENCODER_MULTIPLIER"].is<float>()) ENCODER_MULTIPLIER = config["ENCODER_MULTIPLIER"];

    serialLogln((char*)"Config Set!", 2);
}

#endif