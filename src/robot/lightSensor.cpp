#ifndef CHESSBOT_LIGHT_SENSOR_CPP
#define CHESSBOT_LIGHT_SENSOR_CPP

// Associated Header File
#include "robot/lightSensor.h"
#include "../env.h"

// Built-In Libraries
#include "Arduino.h"

// Custom Libraries
#include "utils/logging.h"
#include "utils/config.h"
#include "utils/timer.h"

short lightArray[4];
//MUST BE MULTIPLE OF DIFF_TICK
const uint8_t COOLDOWN_TIME = 2;
uint8_t cooldown[4] = {0, 0, 0, 0};
float prevTickVals[4] = {-1, -1, -1, -1};
const uint8_t DIFF_TICK = 2;
uint8_t counter = 1;

const short LIMIT_TO_CHANGE = 600;

// Sets the IR (Infrared) Blaster to be able to output
void setupIR() {
    serialLogln("Setting Up Light Sensors...", 2);
    pinMode(RELAY_IR_LED_PIN, OUTPUT);
    for(uint8_t i = 0; i < 20; i++)
    {
        readGarbageVals();
    }
    serialLogln("Light Sensors Setup!", 2);
}

void readGarbageVals() {
    digitalWrite(RELAY_IR_LED_PIN, HIGH);
    delay(50);
    digitalWrite(RELAY_IR_LED_PIN, LOW);
    delay(5);
}

// Turns on the IR Blaster
void activateIR() {
    digitalWrite(RELAY_IR_LED_PIN, HIGH);
}

// Turns off the IR Blaster
void deactivateIR() {
    digitalWrite(RELAY_IR_LED_PIN, LOW);
}

//returns false if no cooldown active, otherwise updates cooldown and returns true 
bool updateCooldown(uint8_t i)
{
    if(cooldown[i] == 0)
    {
        return false;
    }
    cooldown[i]++;
    if(cooldown[i] == COOLDOWN_TIME + 1)
    {
        cooldown[i] = 0;
    }
    serialLogln((char*) "tick on cooldown!", 4);
    return true;
}

//detects the light change. returns true if change, returns false otherwise
bool checkForLightChange(uint8_t i)
{
    //if difference to great and we're not at the start where prevTick is -1, declare that the encoder has changed color, and assign cooldown.
    if(fabs(prevTickVals[i] - lightArray[i]) >= LIMIT_TO_CHANGE && (short) prevTickVals[i] != -1)
    {
        serialLog(prevTickVals[i], 3);
        serialLogln((char*) "", 3);
        serialLogln((char*) "THE DIFFERENCE IS BIG SO WE HAVE CHANGED COLOR GRAHH", 3);
        cooldown[i] = 1;
        return true;
    }
    return false;
}

//updates counter value for us.
void updateCounter() {
    counter++;
    if(counter == DIFF_TICK + 1)
    {
        counter = 1;
    }
}

void startLightReading(bool* onFirstTile, bool* waitingForLight) {
    //for each tick, check if its previous value is too far from what it was before. If it is, consider that the encoder has changed color.
    readLightLevels();
    *waitingForLight = false;

    for(uint8_t i = 0; i < 4; i++)
    {
        //if updating cooldown, skip what's below.
        if(updateCooldown(i))
        {
            continue;
        }
        //if its time to change the tick
        if(counter == DIFF_TICK)
        {
            //only print once
            if(i == 0)
            {
                serialLogln((char*) "Checking tick values!", 4);
            }

            if(checkForLightChange(i))
            {
                //invert value
                onFirstTile[i] = !onFirstTile[i];
            }
            //reset the tick values
            prevTickVals[i] = 0;
        }
        else
        {
            //do DIFF_TICK - 1 because counter can have values 1, 2, 3, 4, and 5, and when count = 5 then we reset. So only 4 samples used.
            prevTickVals[i] += (float) lightArray[i] / (DIFF_TICK - 1);
        }
    }
    //updating counter
    updateCounter();
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
#if LOGGING_LEVEL >= 4
    serialLog("Light Levels: ", 2);
    serialLog(lightArray[0], 2);
    serialLog(" ", 2);
    serialLog(lightArray[1], 2);
    serialLog(" ", 2);
    serialLog(lightArray[2], 2);
    serialLog(" ", 2);
    serialLogln(lightArray[3], 2);
#endif
}

#endif