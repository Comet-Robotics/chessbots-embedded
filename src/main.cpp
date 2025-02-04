#ifndef CHESSBOT_MAIN_CPP
#define CHESSBOT_MAIN_CPP

#include <Arduino.h>
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
    if (getLoggingStatus) Serial.begin(115200);

    // Any setup needed to get bot ready
    setupBot();
    // Create a WiFi network for the laptop to connect to
    connectWiFI();

    if (DO_DRIVE_TEST) startDriveTest();
}

void loop() {
    if (!testConnection()) {
        reconnect();
    }

    if (getServerConnectionStatus()) {
        acceptData();
    }

    if (DO_LIGHT_SENSOR_TEST) logLight();

  	delay(100);
  	timerStep();
}

#endif
