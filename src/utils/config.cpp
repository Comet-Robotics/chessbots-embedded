#ifndef CHESSBOT_CONFIG_CPP
#define CHESSBOT_CONFIG_CPP

// Associated Header File
#include "utils/logging.h"
#include "utils/config.h"

// Built-In Libraries
#include "Arduino.h"

// External Libraries
#include <ArduinoJson.h>

namespace ChessBot
{
    void setConfig(JsonObject config) {
        if (config["MOTOR_A_PIN1"].is<gpio_num_t>()) Config::getInstance().MOTOR_A_PIN1 = config["MOTOR_A_PIN1"];
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

        if (config["MOTOR_A_DRIVE_MULTIPLIER"].is<float>()) {
            logln((char*)"Detected Name Properly", 2);
            logln(config["MOTOR_A_DRIVE_MULTIPLIER"].as<float>(), 3);
            MOTOR_A_DRIVE_MULTIPLIER = config["MOTOR_A_DRIVE_MULTIPLIER"];
        }
        if (config["MOTOR_B_DRIVE_MULTIPLIER"].is<float>()) MOTOR_B_DRIVE_MULTIPLIER = config["MOTOR_B_DRIVE_MULTIPLIER"];

        if (config["ENCODER_MULTIPLIER"].is<float>()) ENCODER_MULTIPLIER = config["ENCODER_MULTIPLIER"];
    }
};

#endif