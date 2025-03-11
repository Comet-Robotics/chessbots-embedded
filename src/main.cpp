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
#include "robot/pid_controller.h"


// Setup gets run at startup
void setup() {
    // Serial port for debugging purposes
    if (LOGGING_LEVEL > 0) Serial.begin(115200);

    //delay(STARTUP_DELAY);
    serialLogln((char*)"Finished Delay!", 2);


    // Any setup needed to get bot ready
    setupBot();

    //For encoder A testing
    //PIDcontroller( kp,  ki,  kd,  min,  max); //For argument reference
    PIDcontroller newControllerA = PIDcontroller(1, 0, 0, -1, +1); //Demo vaues...update with actual encoder ticks
    newControllerA.Compute(1, 1, 0.1); // calling compute function

    //For encoder B testing
    PIDcontroller newControllerB = PIDcontroller(1, 0, 0, -1, +1); //Demo vaues...update with actual encoder ticks
    newControllerB.Compute(1, 1, 0.1); // calling compute function
    // Create a WiFi network for the laptop to connect to
    connectWiFI();

    if (DO_DRIVE_TEST) startDriveTest();
}

// After setup gets run, loop is run over and over as fast ass possible
void loop() {
    // Checks if any timers have expired
    timerStep();

    // Checks whether bot is still connected to WiFi. Reconnect if not
    if (getWiFiConnectionStatus() && !checkWiFiConnection()) reconnectWiFI();
    // Checks whether bot is still connected to the server. Reconnect if not
    if (getServerConnectionStatus() && !checkServerConnection()) reconnectServer();

    // If the bot is connected to the server, check for received data, and accept it if available
    if (getServerConnectionStatus()) acceptData();

    if (DO_LIGHT_SENSOR_TEST) readLight();

    if (DO_ENCODER_TEST) encoderLoop();

    // This delay determines how often the code in loop is run
    // (Forcefully pauses the thread for about the amount of milliseconds passed in)
  	delay(100);
}

// This is used at the end of each file due to the name definition at the beginning
#endif
