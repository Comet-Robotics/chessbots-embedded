#ifndef CHESSBOT_CONFIG_H
#define CHESSBOT_CONFIG_H

// Built-In Libraries
#include "Arduino.h"

// External Libraries
#include <ArduinoJson.h>

namespace ChessBot
{
    class Config {
    private:
        static Config* instance;
        
        Config() { }
        
    public:
        Config(const Config&) = delete;
        Config& operator=(const Config&) = delete;
        
        static Config* getInstance() {
            if (!instance) {
                instance = new Config();
            }
            return instance;
        }

        gpio_num_t MOTOR_A_PIN1 = GPIO_NUM_33;
        gpio_num_t MOTOR_A_PIN2 = GPIO_NUM_38;
        gpio_num_t MOTOR_B_PIN1 = GPIO_NUM_39;
        gpio_num_t MOTOR_B_PIN2 = GPIO_NUM_40;
    
        gpio_num_t ENCODER_A_PIN1 = GPIO_NUM_32;
        gpio_num_t ENCODER_A_PIN2 = GPIO_NUM_31;
        gpio_num_t ENCODER_B_PIN1 = GPIO_NUM_18;
        gpio_num_t ENCODER_B_PIN2 = GPIO_NUM_34;
    
        gpio_num_t RELAY_IR_LED_PIN = GPIO_NUM_15;
        gpio_num_t PHOTODIODE_A_PIN = GPIO_NUM_1;
        gpio_num_t PHOTODIODE_B_PIN = GPIO_NUM_2;
        gpio_num_t PHOTODIODE_C_PIN = GPIO_NUM_4;
        gpio_num_t PHOTODIODE_D_PIN = GPIO_NUM_6;
    
        float WHEEL_DIAMETER_INCHES = 4.375;
    
        float MOTOR_A_DRIVE_MULTIPLIER = 1.0;
        float MOTOR_B_DRIVE_MULTIPLIER = 1.0;
    
        float ENCODER_MULTIPLIER = 1.0;
    };

    Config* Config::instance = nullptr;

    void setConfig(JsonObject config);
};

#endif