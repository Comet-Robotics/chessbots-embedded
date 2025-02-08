#ifndef CHESSBOT_CONFIG_CPP
#define CHESSBOT_CONFIG_CPP

// Associated Header File
#include "utils/config.h"

// Built-In Libraries
#include "Arduino.h"

// External Libraries
#include <ArduinoJson.h>

namespace ChessBot
{
    void setConfig(JsonDocument& packet) {
        if (packet["config"]["MOTOR_A_PIN1"]) MOTOR_A_PIN1 = packet["config"]["MOTOR_A_PIN1"];
        if (packet["config"]["MOTOR_A_PIN2"]) MOTOR_A_PIN2 = packet["config"]["MOTOR_A_PIN2"];
        if (packet["config"]["MOTOR_B_PIN1"]) MOTOR_B_PIN1 = packet["config"]["MOTOR_B_PIN1"];
        if (packet["config"]["MOTOR_B_PIN2"]) MOTOR_B_PIN2 = packet["config"]["MOTOR_B_PIN2"];

        if (packet["config"]["ENCODER_A_PIN1"]) ENCODER_A_PIN1 = packet["config"]["ENCODER_A_PIN1"];
        if (packet["config"]["ENCODER_A_PIN2"]) ENCODER_A_PIN2 = packet["config"]["ENCODER_A_PIN2"];
        if (packet["config"]["ENCODER_B_PIN1"]) ENCODER_B_PIN1 = packet["config"]["ENCODER_B_PIN1"];
        if (packet["config"]["ENCODER_B_PIN2"]) ENCODER_B_PIN2 = packet["config"]["ENCODER_B_PIN2"];

        if (packet["config"]["RELAY_IR_LED_PIN"]) RELAY_IR_LED_PIN = packet["config"]["RELAY_IR_LED_PIN"];
        if (packet["config"]["PHOTODIODE_B_PIN"]) PHOTODIODE_B_PIN = packet["config"]["PHOTODIODE_B_PIN"];
        if (packet["config"]["PHOTODIODE_C_PIN"]) PHOTODIODE_C_PIN = packet["config"]["PHOTODIODE_C_PIN"];
        if (packet["config"]["PHOTODIODE_D_PIN"]) PHOTODIODE_D_PIN = packet["config"]["PHOTODIODE_D_PIN"];
        if (packet["config"]["PHOTODIODE_A_PIN"]) PHOTODIODE_A_PIN = packet["config"]["PHOTODIODE_A_PIN"];

        if (packet["config"]["WHEEL_DIAMETER_INCHES"]) WHEEL_DIAMETER_INCHES = packet["config"]["WHEEL_DIAMETER_INCHES"];

        if (packet["config"]["MOTOR_A_DRIVE_MULTIPLIER"]) MOTOR_A_DRIVE_MULTIPLIER = packet["config"]["MOTOR_A_DRIVE_MULTIPLIER"];
        if (packet["config"]["MOTOR_B_DRIVE_MULTIPLIER"]) MOTOR_B_DRIVE_MULTIPLIER = packet["config"]["MOTOR_B_DRIVE_MULTIPLIER"];

        if (packet["config"]["ENCODER_MULTIPLIER"]) ENCODER_MULTIPLIER = packet["config"]["ENCODER_MULTIPLIER"];
    }
};

#endif