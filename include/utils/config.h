#ifndef CHESSBOT_CONFIG_H
#define CHESSBOT_CONFIG_H

// Built-In Libraries
#include "Arduino.h"

// external Libraries
#include <ArduinoJson.h>

// These variables are declared here, and defined in config.cpp
// config.cpp is the only file that should be modifying these values. Everything else is read-only
extern gpio_num_t MOTOR_A_PIN1;
extern gpio_num_t MOTOR_A_PIN2;
extern gpio_num_t MOTOR_B_PIN1;
extern gpio_num_t MOTOR_B_PIN2;

extern gpio_num_t ENCODER_A_PIN1;
extern gpio_num_t ENCODER_A_PIN2;
extern gpio_num_t ENCODER_B_PIN1;
extern gpio_num_t ENCODER_B_PIN2;

extern gpio_num_t RELAY_IR_LED_PIN;
extern gpio_num_t PHOTODIODE_A_PIN;
extern gpio_num_t PHOTODIODE_B_PIN;
extern gpio_num_t PHOTODIODE_C_PIN;
extern gpio_num_t PHOTODIODE_D_PIN;

extern gpio_num_t BATTERY_VOLTAGE_PIN;

extern float WHEEL_DIAMETER_INCHES;

extern float MOTOR_A_DRIVE_MULTIPLIER;
extern float MOTOR_B_DRIVE_MULTIPLIER;

extern float ENCODER_MULTIPLIER;

void setConfig(JsonObject config);

#endif