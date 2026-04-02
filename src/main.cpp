#include "Arduino.h"

// Custom Libraries
#include "utils/config.h"
#include "utils/logging.h"
#include "utils/timer.h"
#include "utils/status.h"
#include "wifi/wireless.h"
#include "wifi/connection.h"
#include "robot/control/robot.h"
#include "../env.h"
#include "robot/pidController.h"
#include "robot/control/magnet.h"

void loop_test();

unsigned long frame = 0;
unsigned long previousTime = 0; // For loop timing

Robot robot = Robot();

// Setup gets run at startup
void setup() {
    // Serial port for debugging purposes
    if (LOGGING_LEVEL > 0) Serial.begin(115200);

    delay(STARTUP_DELAY);
    serialLogln("Finished Delay!", 2);

    // Create a WiFi network for the laptop to connect to
    if (!RUN_OFFLINE) connectWiFI();

    previousTime = millis() - loopDelayMilliseconds;

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

    // Run control loop
    // TODO change time parameter to be actual delta time, not just delay between loops
    unsigned long deltaTime = millis() - previousTime;
    previousTime = millis();

    loop_test();
    robot.tick(frame, (uint32_t)deltaTime);

    frame++;
}

void loop_test() {
    unsigned long time_seconds = millis() / 1000;

    Coordinate2D goal;
    double rotation = 0;

    if (time_seconds > 10) {
        goal = Coordinate2D(100, 100);
    }

    if (time_seconds > 20) {
        goal = Coordinate2D(0, 0);
    }

    if (time_seconds > 30) {
        goal = Coordinate2D(500, 200);
    }

    robot.drive(goal, rotation);
}