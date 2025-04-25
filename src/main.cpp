// Sets file name into memory (usually to avoid duplicate imports)
#ifndef CHESSBOT_MAIN_CPP
#define CHESSBOT_MAIN_CPP

// Built-In Libraries
#include <Arduino.h>

// Custom Libraries
#include "utils/config.h"
#include "utils/logging.h"
#include "utils/timer.h"
#include "utils/status.h"
#include "wifi/wireless.h"
#include "wifi/connection.h"
#include "robot/control.h"
#include "robot/encoder.h"
#include "../env.h"
#include "robot/pidController.h"

// Setup gets run at startup
void setup() {
    // Serial port for debugging purposes
    if (LOGGING_LEVEL > 0) Serial.begin(115200);

    delay(STARTUP_DELAY);
    serialLogln("Finished Delay!", 2);


    // Any setup needed to get bot ready
    setupBot();

    // Create a WiFi network for the laptop to connect to
    if (!RUN_OFFLINE) connectWiFI();

    if (DO_DRIVE_TEST) startDriveTest();

    if (DO_DRIVE_TICKS_TEST) driveTicks(20000, "NULL");

    if (DO_HARDWARE_TEST) timerDelay(5000, &startMotorAndEncoderTest);
}

// After setup gets run, loop is run over and over as fast ass possible
void loop() {
    // Checks if any timers have expired
    timerStep();

    if (!RUN_OFFLINE) {
        // Checks whether bot is still connected to WiFi. Reconnect if not
        if (getWiFiConnectionStatus() && !checkWiFiConnection()) reconnectWiFI();
        // Checks whether bot is still connected to the server. Reconnect if not
        if (getServerConnectionStatus() && !checkServerConnection()) reconnectServer();

        // If the bot is connected to the server, check for received data, and accept it if available
        if (getServerConnectionStatus()) acceptData();
    }

    // Run control loop
    controlLoop();

    // This delay determines how often the code in loop is run
    // (Forcefully pauses the thread for about the amount of milliseconds passed in)
  	delay(loopDelayMilliseconds);
}

// This is used at the end of each file due to the name definition at the beginning
#endif
