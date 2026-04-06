#include <Arduino.h>

#include "../env.h"
#include "robot/control/magnet.h"
#include "robot/control/robot.h"
#include "robot/pidController.h"
#include "tests.h"
#include "utils/config.h"
#include "utils/logging.h"
#include "utils/status.h"
#include "utils/timer.h"
#include "wifi/connection.h"
#include "wifi/wireless.h"

uint32_t frame = 0;

uint32_t previous_time = 0;

Robot robot = Robot();

// Setup gets run at startup
void setup() {
    // Serial port for debugging purposes
    if (LOGGING_LEVEL > 0) Serial.begin(115200);

    delay(STARTUP_DELAY);
    serial_printf(DebugLevel::DEBUG, "Finished Delay!\n");

    // Create a WiFi network for the laptop to connect to
    if (!RUN_OFFLINE) connectWiFI();

    previous_time = micros();
    // robot.center();
}

// After setup gets run, loop is run over and over as fast ass possible
void loop() {
    // Checks if any timers have expired
    timerStep();

    if (!RUN_OFFLINE) 
    {
        // Checks whether bot is still connected to WiFi. Reconnect if not
        if (getWiFiConnectionStatus() && !checkWiFiConnection()) reconnectWiFI();
        // Checks whether bot is still connected to the server. Reconnect if not
        if (getServerConnectionStatus() && !checkServerConnection()) reconnectServer();

        // If the bot is connected to the server, check for received data, and accept it if available
        if (getServerConnectionStatus()) acceptData();
    }

    uint32_t delta = micros() - previous_time;
    previous_time = micros();

    square_test(robot);
    robot.tick(frame, delta);

    frame++;
}