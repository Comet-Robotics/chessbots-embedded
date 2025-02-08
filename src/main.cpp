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
#include "../env.h"

using namespace ChessBot;

void setup() {
    // Serial port for debugging purposes
    if (LOGGING_LEVEL > 0) Serial.begin(115200);

    // Any setup needed to get bot ready
    setupBot();
    // Create a WiFi network for the laptop to connect to
    connectWiFI();

    if (DO_DRIVE_TEST) startDriveTest();
}

void loop() {
    // Checks whether bot is still connected to the server. Reconnect if not
    if (!testConnection()) {
        reconnect();
    }

    // If the bot is connected for the server, check for received data, and accept if if available
    if (getServerConnectionStatus()) {
        acceptData();
    }

    if (DO_LIGHT_SENSOR_TEST) readLight();

  	delay(100);
  	timerStep();
}

#endif
