#ifndef CHESSBOT_CONTROL_CPP
#define CHESSBOT_CONTROL_CPP

// Associated Header File
#include "robot/control.h"

// Built-In Libraries
#include "Arduino.h"

// Custom Libraries
#include "utils/logging.h"
#include "utils/timer.h"
#include "utils/config.h"
#include "robot/motor.h"
#include "robot/lightSensor.h"
#include "wifi/connection.h"
#include "robot/encoder_new.h"

bool setupDrive = true;
bool topLeftEncodeVal = false;
bool topRightEncodeVal = false;

uint8_t iteration = 0;
uint8_t maxTicks = -1;

const uint8_t Top_Left_Encoder_Index = 1;
const uint8_t Top_Right_Encoder_Index = 2;
const uint8_t Bottom_Left_Encoder_Index = 3;
const uint8_t Bottom_Right_Encoder_Index = 0;

// Sets up all the aspects needed for the bot to work
void setupBot() {
    serialLogln((char*)"Setting Up Bot...", 2);
    setupMotors();
    setupIR();
    setupEncodersNew();
    serialLogln((char*)"Bot Set Up!", 2);
}

// Drives a specific amount of tiles (WIP)
void drive(float tiles) {

}

// Drives the wheels according to the powers set. Negative is backwards, Positive forwards
void drive(float leftPower, float rightPower, std::string id) {
    setLeftPower(leftPower);
    setRightPower(rightPower);

    // Logs the Drive values for debugging purposes
    serialLog((char*)"Drive: ", 2);
    serialLog(leftPower, 2);
    serialLog((char*)", ", 2);
    serialLogln(rightPower, 2);
    //we only send null as id during our test drive. The only other time this drive method is called will be
    //when the server sends it, meaning it will have an id to send back.
    if(id != "NULL")
    {
        createAndSendPacket(2, "success", id);   
    }
}

// Stops the bot in its tracks
void stop() {
    setLeftPower(0);
    setRightPower(0);

    serialLogln((char*)"Bot Stopped!", 2);
}

// Reads in the light value of all light sensors
void readLight(bool* onFirstTile) {
    // The Infrared Blaster must be activated first to get a clear reading
    activateIR();
    //this way, we can pass in a parameter to timerDelay as well, but we don't have to
    timerDelay(50, std::bind(startLightReading, onFirstTile));
}

// Tests the motors. This turns the motors on.
void startDriveTest() {
    drive(-0.5f, -0.5f, "NULL");
    timerDelay(2000, &driveTestOff);
}

//drives until the first two encoders are considered to hit a new value
bool driveRobotUntilNewTile(bool* onFirstTile) 
{
    //intially set this up
    if(setupDrive)
    {
        drive(-0.5f, -0.5f, "NULL");
        //assign values here, will detect when they change
        topLeftEncodeVal = onFirstTile[Top_Left_Encoder_Index];
        topRightEncodeVal = onFirstTile[Top_Right_Encoder_Index];
    }
    setupDrive = false;
    //on change, stop
    if(onFirstTile[Top_Left_Encoder_Index] != topLeftEncodeVal || onFirstTile[Top_Right_Encoder_Index] != topRightEncodeVal)
    {
        stop();
        return true;
    }
    return false;
}

void setReverseTicks(uint8_t max_ticks)
{
    maxTicks = max_ticks;
}

bool reverseRobotXTicks()
{
    if(iteration == 0)
    {
        drive(0.5f, 0.5f, "NULL");
    }
    if(iteration == maxTicks)
    {
        iteration = 0;
        stop();
        return false;
    }
    iteration++;
    return true;
}

// Tests the motors. This turns the motors off.
void driveTestOff() {
    stop();
    timerDelay(2000, &startDriveTest);
}

#endif