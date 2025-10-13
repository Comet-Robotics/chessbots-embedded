#ifndef CHESSBOT_CONFIG_CPP
#define CHESSBOT_CONFIG_CPP

// Associated Header File
#include "utils/config.h"

// Built-In Libraries
#include "Arduino.h"

// Custom Libraries
#include "utils/logging.h"

// External Libraries
#include <ArduinoJson.h>

int loopDelayMilliseconds = 20;

gpio_num_t MOTOR_A_PIN1 = GPIO_NUM_33;
gpio_num_t MOTOR_A_PIN2 = GPIO_NUM_38;
gpio_num_t MOTOR_B_PIN1 = GPIO_NUM_39;
gpio_num_t MOTOR_B_PIN2 = GPIO_NUM_40;

gpio_num_t ENCODER_A_PIN1 = GPIO_NUM_18;
gpio_num_t ENCODER_A_PIN2 = GPIO_NUM_21;
gpio_num_t ENCODER_B_PIN1 = GPIO_NUM_16;
gpio_num_t ENCODER_B_PIN2 = GPIO_NUM_17;

gpio_num_t RELAY_IR_LED_PIN = GPIO_NUM_11;
gpio_num_t PHOTODIODE_A_PIN = GPIO_NUM_1;
gpio_num_t PHOTODIODE_B_PIN = GPIO_NUM_2;
gpio_num_t PHOTODIODE_C_PIN = GPIO_NUM_4;
gpio_num_t PHOTODIODE_D_PIN = GPIO_NUM_6;

gpio_num_t ONBOARD_LED_PIN = GPIO_NUM_15;
gpio_num_t BATTERY_VOLTAGE_PIN = GPIO_NUM_10;

int TICKS_PER_ROTATION = 12000;
float TRACK_WIDTH_INCHES = 8.29;
float WHEEL_DIAMETER_INCHES = 4.75;
float THEORETICAL_MAX_VELOCITY_TPS = 63000;
float THEORETICAL_MAX_ACCELERATION_TPSPS = 16000;
float VELOCITY_LIMIT_TPS = 40000;
float ACCELERATION_LIMIT_TPSPS = 10000;
float MIN_MOTOR_POWER = 0.12; // Minimum motor power to elicit motor response, empirically determined
float TILES_TO_TICKS = 2*12*TICKS_PER_ROTATION/(WHEEL_DIAMETER_INCHES*M_PI);

int MAGNET_CCW_IS_POSITIVE = 1; // Set to 1 if counterclockwise rotation is positive, -1 if clockwise rotation is positive

float PID_POSITION_TOLERANCE = 100;
float PID_VELOCITY_TOLERANCE = 6000;

void setConfig(JsonObject config) {
    serialLogln("Setting Config...", 2);

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

    if (config["TICKS_PER_ROTATION"].is<int>()) TICKS_PER_ROTATION = config["TICKS_PER_ROTATION"];
    if (config["TRACK_WIDTH_INCHES"].is<float>()) TRACK_WIDTH_INCHES = config["TRACK_WIDTH_INCHES"];
    if (config["WHEEL_DIAMETER_INCHES"].is<float>()) WHEEL_DIAMETER_INCHES = config["WHEEL_DIAMETER_INCHES"];
    if (config["THEORETICAL_MAX_VELOCITY_TPS"].is<float>()) THEORETICAL_MAX_VELOCITY_TPS = config["THEORETICAL_MAX_VELOCITY_TPS"];
    if (config["THEORETICAL_MAX_ACCELERATION_TPSPS"].is<float>()) THEORETICAL_MAX_ACCELERATION_TPSPS = config["THEORETICAL_MAX_ACCELERATION_TPSPS"];
    if (config["MIN_MOTOR_POWER"].is<float>()) MIN_MOTOR_POWER = config["MIN_MOTOR_POWER"];

    if (config["MAGNET_CCW_IS_POSITIVE"].is<int>()) MAGNET_CCW_IS_POSITIVE = config["MAGNET_CCW_IS_POSITIVE"];

    serialLogln("Config Set!", 2);
}

#endif