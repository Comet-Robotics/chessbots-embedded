#ifndef CHESSBOT_LIGHT_SENSOR_CPP
#define CHESSBOT_LIGHT_SENSOR_CPP

// Associated Header File
#include "robot/lightSensor.h"

// Built-In Libraries
#include "Arduino.h"

// Custom Libraries
#include "utils/logging.h"
#include "utils/config.h"
#include "utils/timer.h"

int lightArray[4];
const uint8_t COOLDOWN_TIME = 5;
uint8_t cooldown[4] = {0, 0, 0, 0};
float prevTickVals[4] = {-1, -1, -1, -1};
const uint8_t DIFF_TICK = 5;
uint8_t counter = 1;

// Sets the IR (Infrared) Blaster to be able to output
void setupIR() {
    serialLogln((char*)"Setting Up Light Sensors...", 2);
    pinMode(RELAY_IR_LED_PIN, OUTPUT);
    serialLogln((char*)"Light Sensors Setup!", 2);
}

// Turns on the IR Blaster
void activateIR() {
    digitalWrite(RELAY_IR_LED_PIN, HIGH);
}

// Turns off the IR Blaster
void deactivateIR() {
    digitalWrite(RELAY_IR_LED_PIN, LOW);
}

void startLightReading() {
    // The Infrared Blaster must be activated first to get a clear reading
    activateIR();
    // To give time for the IR Blaster to activate, only reads light values after 50 milliseconds
    timerDelay(50, &readLightLevels);
    
    if(counter == DIFF_TICK)
    {
        serialLogln((char*) "Time to update ticks!", 2);
        //for each tick, check if its previous value is too far from what it was before. If it is, consider that the encoder has changed color.
        for(int i = 0; i < 4; i++)
        {
            if(cooldown[i] != 0)
            {
                cooldown[i]++;
                if(cooldown[i] == COOLDOWN_TIME)
                {
                    cooldown[i] = 0;
                }
                serialLogln((char*) "tick on cooldown!", 2);
                continue;
            }
            if(abs(prevTickVals[i] - lightArray[i]) >= 200 && prevTickVals[i] != -1)
            {
                serialLogln((char*) "THE DIFFERENCE IS BIG SO WE HAVE CHANGED COLOR GRAHH", 2);
                serialLog((char*) "bruh", 2);
                serialLog(prevTickVals[i], 2);
                serialLogln((char*) "bruh", 2);
                cooldown[i] = 1;
            }
            prevTickVals[i] = 0;
        }
    }
    else
    {
        serialLogln((char*) "update sum in the meantime", 2);
        //now, update it for each encoder.
        for(int i = 0; i < 4; i++)
        {
            if(cooldown[i] != 0)
            {
                cooldown[i]++;
                if(cooldown[i] == COOLDOWN_TIME)
                {
                    cooldown[i] = 0;
                }
                serialLogln((char*) "tick on cooldown!", 2);
                continue;
            }
            prevTickVals[i] += (float) lightArray[i] / (DIFF_TICK - 1);
            serialLog(prevTickVals[i], 2);
            serialLogln((char*) " - Val", 2);
        }
        std::string finalStr = "At counter " + std::to_string(counter) + ", the current fraction is " + std::to_string(prevTickVals[0]) + " for light sensor 0.";
        serialLogln(finalStr.c_str(), 4);
    }

    counter++;
    if(counter == DIFF_TICK + 1)
    {
        counter = 1;
    }
}

void readLightLevels() {
    // Reads the values on the light sensor pins
    lightArray[0] = analogRead(PHOTODIODE_A_PIN);
    lightArray[1] = analogRead(PHOTODIODE_B_PIN);
    lightArray[2] = analogRead(PHOTODIODE_C_PIN);
    lightArray[3] = analogRead(PHOTODIODE_D_PIN);

    // Deactivates the IR Blaster after reading to save battery power
    deactivateIR();

    // Logs the values for debugging purposes
    serialLog((char*)"Light Levels: ", 4);
    serialLog(lightArray[0], 4);
    serialLog((char*)" ", 4);
    serialLog(lightArray[1], 4);
    serialLog((char*)" ", 4);
    serialLog(lightArray[2], 4);
    serialLog((char*)" ", 4);
    serialLogln(lightArray[3], 4);
}

#endif