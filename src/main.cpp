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

//may want to make this value accessible from other files? Because we are going to be passing it around a lot
bool onFirstTile[4] = {false, false, false, false};

bool drivingRobot = true;

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


#if DO_LIGHT_SENSOR_TEST
    readLight(onFirstTile);
    //know our char will be 4 bits
    char vals[5];
    //read the booleans as char
    for(uint8_t i = 0; i < 4; i++)
    {
        vals[i] = onFirstTile[i] ? '1' : '0';
    }
    //must null terminate
    vals[4] = '\0';
    serialLog((char*) "Light statuses: ", 4);
    serialLogln((char*) vals, 2);
#endif

    
    if(drivingRobot)
    {
        bool finished = driveRobotUntilNewTile(onFirstTile);
        if(finished)
        {
            drivingRobot = false;
            serialLogln((char*) "STOP DRIVING!", 4);
        }
        else
        {
            serialLogln((char*) "DRIVING", 4);
        }
    }

    if (DO_ENCODER_TEST) encoderLoop();

    // This delay determines how often the code in loop is run
    // (Forcefully pauses the thread for about the amount of milliseconds passed in) 
  	delay(100);
}

// This is used at the end of each file due to the name definition at the beginning
#endif
