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

bool firstEncoderVal = false;
bool secondEncoderVal = false;

bool driveUntilChange = false;
uint8_t iteration = 0;
uint8_t maxTicks = 0;
bool movingXTicks = false;

//Here are the possible values:
    //0 = no encoder leading.
    //1 = left encoder leading.
    //2 = right encoder leading
uint8_t leadingEncoder = 0;
//Ticks it tackes for back encoder to reach tile change after the front tile does.
uint8_t backEncoderDist = 0;

const uint8_t Top_Left_Encoder_Index = 1;
const uint8_t Top_Right_Encoder_Index = 2;
const uint8_t Bottom_Left_Encoder_Index = 3;
const uint8_t Bottom_Right_Encoder_Index = 0;

//put this in manually for each bot. Dist between the two front encoders, or the two back encoders.
const float encoderDist = 0.07;

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

void createDriveUntilNewTile(bool* onFirstTile)
{
    drive(-0.5f, -0.5f, "NULL");
    //assign values here, will detect when they change
    firstEncoderVal = onFirstTile[Top_Left_Encoder_Index];
    secondEncoderVal = onFirstTile[Top_Right_Encoder_Index];
    driveUntilChange = true;
}

//drives until the first two encoders are considered to hit a new value.
//Returns a status bit, with the correspnding values:
    // 0 - drive is not active
    // 1 - drive is active
    // 2 - just at this moment, we have reached our destination and are going to begin reversing now.
    // 3 - we are now currently reversing the desired ticks
    // 4 - we have now reached the desired ticks.
uint8_t driveUntilNewTile(bool* onFirstTile) 
{
    if(driveUntilChange)
    {
        bool leftEncoderChange = onFirstTile[Top_Left_Encoder_Index] != firstEncoderVal;
        bool rightEncoderChange = onFirstTile[Top_Right_Encoder_Index] != secondEncoderVal;
        if(leftEncoderChange || rightEncoderChange)
        {
            //when both cross, we done.
            if(leftEncoderChange && rightEncoderChange)
            {
                stop();
                driveUntilChange = false;
                beginXTicksDrive(backEncoderDist, true);


                serialLog((char*) " Encoder in front is gonna be: ", 2);
                serialLogln(leadingEncoder, 2);
                serialLog((char*) "And distance back encoder was behind is: ", 2);
                serialLogln(backEncoderDist, 2);

                backEncoderDist = 0;
                leadingEncoder = 0;
                return 2;    
            }
            //otherwise, want to see which one crossed. This is cond1 XOR cond2 btw.
            else if(backEncoderDist == 0)
            {
                //if the left one has crossed but the right hasn't, then just label that's the one crossing now.
                if(leftEncoderChange)
                {
                    leadingEncoder = 1;
                }
                else
                {
                    leadingEncoder = 2;
                }
                backEncoderDist++;
            }
            //otherwise, continue moving the back encoder and just make sure we update its dist until it stops moving
            else
            {
                backEncoderDist++;
            }
        }
        return 1;
    }
    //this will only be true when we're reversing
    else if(movingXTicks)
    {
        moveRobotXTicks();
        //if we're still moving, then return a different status
        if(movingXTicks)
        {
            return 3;
        }
        else
        {
            return 4;
        }
    }
    return 0;
}

void beginXTicksDrive(uint8_t max_ticks, bool inReverse)
{
    maxTicks = max_ticks;
    movingXTicks = true;
    if(inReverse)
    {
        //for some reason, positive drive values move it backward? Or maybe I don't understand the direction of the robot.
        drive(0.5f, 0.5f, "NULL");
    }
    else
    {
        drive(-0.5f, -0.5f, "NULL");
    }
}

void moveRobotXTicks()
{
    if(movingXTicks)
    {
        if(iteration == maxTicks)
        {
            iteration = -1;
            movingXTicks = false;
            stop();
        }
        iteration++;
    }
}

// Tests the motors. This turns the motors off.
void driveTestOff() {
    stop();
    timerDelay(2000, &startDriveTest);
}

#endif