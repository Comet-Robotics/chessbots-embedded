#ifndef CHESSBOT_LIGHT_SENSOR_CPP
#define CHESSBOT_LIGHT_SENSOR_CPP

#include "robot/lightSensor.h"

#include "Arduino.h"
#include "utils/logging.h"
#include "utils/config.h"

namespace ChessBot
{
    int lightArray[4];

    void setupIR() {
        pinMode(RELAY_IR_LED_PIN, OUTPUT);
    }

    void activateIR() {
        digitalWrite(RELAY_IR_LED_PIN, HIGH);
    }

    void deactivateIR() {
        digitalWrite(RELAY_IR_LED_PIN, LOW);
    }

    void readLightLevels(int lightArray[]) {
        lightArray[0] = analogRead(PHOTODIODE_A_PIN);
        lightArray[1] = analogRead(PHOTODIODE_B_PIN);
        lightArray[2] = analogRead(PHOTODIODE_C_PIN);
        lightArray[3] = analogRead(PHOTODIODE_D_PIN);
    }

    void logLightLevels() {
        readLightLevels(lightArray);

        log((char*)"Light Levels: ");
        log(lightArray[0]);
        log((char*)" ");
        log(lightArray[1]);
        log((char*)" ");
        log(lightArray[2]);
        log((char*)" ");
        logln(lightArray[3]);
    }
};

#endif