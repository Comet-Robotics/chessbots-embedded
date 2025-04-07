// Sets file name into memory (usually to avoid duplicate imports)
#ifndef CHESSBOT_MAIN_CPP
#define CHESSBOT_MAIN_CPP

// Built-In Libraries
#include <Arduino.h>

// Custom Libraries
#include "utils/logging.h"
#include "utils/timer.h"
#include "utils/status.h"
#include "wifi/wireless.h"
#include "wifi/connection.h"
#include "robot/control.h"
#include "robot/encoder_new.h"
#include "../env.h"

// int currentTick = 0;
// const int MAX_TICK = 20;
// int lightSum[4] = {0, 0, 0, 0};
// int prevSum[4] = {0, 0, 0, 0};

const uint8_t DIFF_TICK = 5;
int prevTick[4] = {-1, -1, -1, -1};
uint8_t counter = 0;

// Setup gets run at startup
void setup() {
    // Serial port for debugging purposes
    if (LOGGING_LEVEL > 0) Serial.begin(115200);

    delay(STARTUP_DELAY);
    serialLogln((char*)"Finished Delay!", 2);

    // Any setup needed to get bot ready
    setupBot();
    // Create a WiFi network for the laptop to connect to
    connectWiFI();

    if (DO_DRIVE_TEST) startDriveTest();
}

// After setup gets run, loop is run over and over as fast ass possible
void loop() {
    // Checks if any timers have expired
    timerStep();
    
    // Checks whether bot is still connected to WiFi. Reconnect if not
    if (getWiFiConnectionStatus() && !checkWiFiConnection())
    {
        reconnectWiFI();
    }

    // Checks whether bot is still connected to the server. Reconnect if not
    if (getServerConnectionStatus() && !checkServerConnection()) reconnectServer();

    // If the bot is connected to the server, check for received data, and accept it if available
    if (getServerConnectionStatus()) acceptData();

    if (DO_LIGHT_SENSOR_TEST) readLight(counter, prevTick, DIFF_TICK);

    if (DO_ENCODER_TEST) encoderLoop();

    // This delay determines how often the code in loop is run
    // (Forcefully pauses the thread for about the amount of milliseconds passed in) 
  	delay(100);
    if(counter == DIFF_TICK)
    {
        counter = 0;
    }
    counter++;
}

// This is used at the end of each file due to the name definition at the beginning
#endif
