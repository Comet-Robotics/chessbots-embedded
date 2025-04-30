#ifndef CHESSBOT_CONTROL_CPP
#define CHESSBOT_CONTROL_CPP

// Associated Header File
#include "robot/control.h"
#include "robot/trapezoidalProfile.h"

// Built-In Libraries
#include "Arduino.h"

// Custom Libraries
#include "utils/logging.h"
#include "utils/timer.h"
#include "utils/config.h"
#include "robot/motor.h"
#include "robot/lightSensor.h"
#include "wifi/connection.h"
#include "robot/encoder.h"
#include "robot/pidController.h"
#include "robot/driveTest.h"
#include "../../env.h"
#include <algorithm>

//PLEASE ONLY USE CHESSBOT #4 FOR TESTING
PIDController encoderAVelocityController(0.00008, 0.0000035, 0.000001, -1, +1); //Blue
PIDController encoderBVelocityController(0.00008, 0.0000035, 0.000001, -1, +1); //Red

int encoderATarget = 0;
int encoderBTarget = 0;
int prevPositionA = 0;
int prevPositionB = 0;

MotionProfile profileA = {MAX_VELOCITY_TPS, MAX_ACCELERATION_TPSPS, 0, 0, 0, 0}; // maxVelocity, maxAcceleration, currentPosition, currentVelocity, targetPosition, targetVelocity
MotionProfile profileB = {MAX_VELOCITY_TPS, MAX_ACCELERATION_TPSPS, 0, 0, 0, 0}; // maxVelocity, maxAcceleration, currentPosition, currentVelocity, targetPosition, targetVelocity

boolean testEncoderPID_value = false;
void testEncoderPID()
{
    serialLogln("Changing encoder PID setpoint!", 2);
    if (!testEncoderPID_value)
    {
        testEncoderPID_value = true;
        encoderATarget = encoderBTarget = TICKS_PER_ROTATION*5;
    }
    else
    {
        testEncoderPID_value = false;
        encoderATarget = encoderBTarget = 0;
    }
}

int testTurn_angle = 0;
void testTurn()
{
    serialLog("Changing destination angle to ", 2);
    serialLog(testTurn_angle, 2);
    testTurn_angle = (testTurn_angle + 90) % 360;
    turn(M_PI / 2, "NULL");
    serialLog(" (", 2);
    serialLog(encoderATarget, 2);
    serialLog(", ", 2);
    serialLog(encoderBTarget, 2);
    serialLogln(")", 2);
}

bool firstEncoderVal = false;
bool secondEncoderVal = false;

bool driveUntilChange = false;
int iteration = 0;
int maxTicks = 0;
bool movingXTicks = false;

//Here are the possible values:
    //0 = no encoder leading.
    //1 = left encoder leading.
    //2 = right encoder leading
uint8_t leadingEncoder = 0;
//Ticks it tackes for back encoder to reach tile change after the front tile does.
int backEncoderDist = 0;
bool leftEncoderChange = false;
bool rightEncoderChange = false;

const uint8_t Top_Left_Encoder_Index = 0;
const uint8_t Top_Right_Encoder_Index = 1;
const uint8_t Bottom_Left_Encoder_Index = 2;
const uint8_t Bottom_Right_Encoder_Index = 3;

//put this in manually for each bot. Dist between the two front encoders, or the two back encoders.
const float encoderDist = 0.07;

// Sets up all the aspects needed for the bot to work
void setupBot() {
    serialLogln("Setting Up Bot...", 2);

    pinMode(ONBOARD_LED_PIN, OUTPUT);
    digitalWrite(ONBOARD_LED_PIN, HIGH);

    setupMotors();
    setupIR();
    setupEncodersNew();
    serialLogln("Bot Set Up!", 2);

    encoderAVelocityController.Reset();
    encoderBVelocityController.Reset();

    if (DO_PID_TEST) {
        testEncoderPID();
        timerInterval(8000, &testEncoderPID);
    }
    
    if (DO_TURN_TEST) {
        testTurn();
        timerInterval(5000, &testTurn);
    }
}

// + (0.0001 * desiredVelocityA)
// Manages control loop (loopDelayMs is for reference)
void controlLoop(int loopDelayMs) {
    if (DO_LIGHT_SENSOR_TEST)
        // readLight();

    if (DO_ENCODER_TEST)
        encoderLoop();

    if (DO_PID) {
        //delay(5000); // Delay to prevent CPU overload
        double loopDelaySeconds = ((double) loopDelayMs) / 1000;

        int currentPositionEncoderA = readLeftEncoder();
        int currentPositionEncoderB = readRightEncoder();

        profileA.targetPosition = encoderATarget;
        profileA.currentPosition = currentPositionEncoderA;

        profileB.targetPosition = encoderBTarget;
        profileB.currentPosition = currentPositionEncoderB;
        
        double currentVelocityA = (currentPositionEncoderA - prevPositionA) / loopDelaySeconds;
        profileA.currentVelocity = currentVelocityA;
        
        double currentVelocityB = (currentPositionEncoderB - prevPositionB) / loopDelaySeconds;
        profileB.currentVelocity = currentVelocityB;
        
        double desiredVelocityA = updateTrapezoidalProfile(profileA, loopDelaySeconds);
        double desiredVelocityB = updateTrapezoidalProfile(profileB, loopDelaySeconds);
        
        prevPositionA = currentPositionEncoderA;        
        prevPositionB = currentPositionEncoderB;

        double leftFeedForward = desiredVelocityA / MAX_VELOCITY_TPS;
        double rightFeedForward = desiredVelocityB / MAX_VELOCITY_TPS;

        double leftMotorPower = encoderAVelocityController.Compute(desiredVelocityA, currentVelocityA, loopDelaySeconds) + leftFeedForward;
        double rightMotorPower = encoderBVelocityController.Compute(desiredVelocityB, currentVelocityB, loopDelaySeconds) + rightFeedForward;

        if (leftMotorPower > 1) leftMotorPower = 1;
        if (leftMotorPower < -1) leftMotorPower = -1;
        if (rightMotorPower > 1) rightMotorPower = 1;
        if (rightMotorPower < -1) rightMotorPower = -1;

        serialLog(currentPositionEncoderA, 3);
        serialLog(",", 3);
        serialLog(currentPositionEncoderB, 3);
        serialLog(",", 3);
        serialLog((float) desiredVelocityA, 3);
        serialLog(",", 3);
        serialLog((float) desiredVelocityB, 3);
        serialLog(",", 3);
        serialLog((float) currentVelocityA, 3);
        serialLog(",", 3);
        serialLog((float) currentVelocityB, 3);
        serialLog(",", 3);
        serialLog((float) leftMotorPower, 3);
        serialLog(",", 3);
        serialLog((float) rightMotorPower, 3);
        serialLog(",", 3);
        serialLog(encoderATarget, 3);
        serialLog(",", 3);
        serialLog(encoderBTarget, 3); // TODO log results of trapezoidal profile into csv (on motor value graph)
        serialLog(",", 3);
        serialLogln((float) loopDelaySeconds, 3);

        drive(
            leftMotorPower, // leftMotorPower,
            rightMotorPower, // rightMotorPower,
            "NULL"
        );
    }
}

// Drives a specific amount of tiles (WIP)
void drive(float tiles) {

}

void driveTicks(int tickDistance, std::string id)
{
    encoderATarget = readLeftEncoder() + tickDistance;
    encoderBTarget = readRightEncoder() - tickDistance;

}

// Drives the wheels according to the powers set. Negative is backwards, Positive forwards
void drive(float leftPower, float rightPower, std::string id) {
    // TODO: maybe move to motor.cpp?
    float minPower = 0.16;
    if (leftPower < minPower && leftPower > -minPower)
    {
        leftPower = 0;
    }
    if (rightPower < minPower && rightPower > -minPower)
    {
        rightPower = 0;
    }

    setLeftPower(leftPower);
    setRightPower(rightPower);

    //we only send null as id during our test drive. The only other time this drive method is called will be
    //when the server sends it, meaning it will have an id to send back.
    if(id != "NULL")
    {
        createAndSendPacket(2, "success", id);   
    }
}

void turn(float angleRadians, std::string id) {
    float offsetInches = TRACK_WIDTH_INCHES * angleRadians / 2;
    int offsetTicks = (int) (offsetInches / (WHEEL_DIAMETER_INCHES * M_PI) * TICKS_PER_ROTATION);

    encoderATarget -= offsetTicks;
    encoderBTarget += offsetTicks;

    // TODO wait for pid to reach target
    if (id != "NULL")
    {
        createAndSendPacket(2, "success", id);
    }
}

// Stops the bot in its tracks
void stop() {
    setLeftPower(0);
    setRightPower(0);

    serialLogln("Bot Stopped!", 2);
}

bool waitingForLight = false;

// Reads in the light value of all light sensors
void readLight(bool* onFirstTile) {
    if(!waitingForLight)
    {
        // The Infrared Blaster must be activated first to get a clear reading
        activateIR();
        waitingForLight = true;
        //this way, we can pass in a parameter to timerDelay as well, but we don't have to
        timerDelay(20, std::bind(startLightReading, onFirstTile, &waitingForLight));
    }
}

// Test motor and encoder synchronization
void startMotorAndEncoderTest() {
    (new MotorEncoderTest())->startMotorAndEncoderTest();
}

// Tests the motors. This turns the motors on.
void startDriveTest() {
    drive(0.5f, 0.5f, "NULL");
    timerDelay(2000, &driveTestOff);
}

void createDriveUntilNewTile(bool* onFirstTile)
{
    drive(0.5f, 0.5f, "NULL");
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
        //if we already changed it, don't change it back again
        leftEncoderChange = leftEncoderChange || (onFirstTile[Top_Left_Encoder_Index] != firstEncoderVal);
        rightEncoderChange = rightEncoderChange || (onFirstTile[Top_Right_Encoder_Index] != secondEncoderVal);
        if(leftEncoderChange || rightEncoderChange)
        {
            //when both cross, we done.
            if(leftEncoderChange && rightEncoderChange)
            {
                stop();
                driveUntilChange = false;
                if(leadingEncoder != 0)
                {
                    float multiplier = 0.5;
                    beginXTicksDrive(leadingEncoder, backEncoderDist * multiplier, true);
                }

                serialLog((char*) " Encoder in front is gonna be: ", 2);
                serialLogln(leadingEncoder, 2);
                serialLog((char*) "And distance back encoder was behind is: ", 2);
                serialLogln(backEncoderDist, 2);

                backEncoderDist = 0;
                leadingEncoder = 0;
                //if the encoder returned 0, no need to reverse so skip to status 4.
                return (leadingEncoder == 0) ? 4 : 2;    
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

void beginXTicksDrive(uint8_t leadingEncoderLabel, int max_ticks, bool inReverse)
{
    //for some reason, positive drive values move it backward? Or maybe I don't understand the direction of the robot.
    maxTicks = max_ticks;
    movingXTicks = true;
    //so this is actually -1 if going in reverse, if going forward it will be 1.
    //We want to use arithmetic instead to make it faster than say if we did conditionals
    int8_t negativeMultiplier = inReverse * -2 + 1;
    //so when the label is 2, 2 - 2 = 0, good. When label is 1, 2 - 1 is 1, good.
    uint8_t leftEncoderMultiplier = 2 - leadingEncoderLabel;
    //when label is 2, 2 - 1 is 1, good. When label is 1, 1 - 1 is 0, good.
    uint8_t rightEncoderMultiplier = leadingEncoderLabel - 1;

    drive(0.5 * negativeMultiplier * leftEncoderMultiplier, 0.5 * negativeMultiplier * rightEncoderMultiplier, "NULL");
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