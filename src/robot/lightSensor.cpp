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

// Sets the IR (Infrared) Blaster to be able to output
void setupIR() {
    log((char*)"Setting Up Light Sensors...", 2);
    pinMode(RELAY_IR_LED_PIN, OUTPUT);
    log((char*)"Light Sensors Setup!", 2);
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
    log((char*)"Light Levels: ", 4);
    log(lightArray[0], 4);
    log((char*)" ", 4);
    log(lightArray[1], 4);
    log((char*)" ", 4);
    log(lightArray[2], 4);
    log((char*)" ", 4);
    logln(lightArray[3], 4);
}

#endif