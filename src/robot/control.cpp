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
int currentEncoderA = -1;
int currentEncoderB = -1;
int startEncoderAPos = -1;
int startEncoderBPos = -1;

bool forwardAligning = true;

//just measured, its 5.9 centimeters, or .059 meters
const float TIRE_RADIUS = 0.059;
//circumference equals pi * diameter. In meters
const float TIRE_CIRCUMFERENCE = M_PI * 2 * TIRE_RADIUS;
//multiply any ticks by this ratio to get distance in meters
const float TICK_TO_METERS = TIRE_CIRCUMFERENCE / TICKS_PER_ROTATION;

MotionProfile profileA = {MAX_VELOCITY_TPS, MAX_ACCELERATION_TPSPS, 0, 0, 0, 0}; // maxVelocity, maxAcceleration, currentPosition, currentVelocity, targetPosition, targetVelocity
MotionProfile profileB = {MAX_VELOCITY_TPS, MAX_ACCELERATION_TPSPS, 0, 0, 0, 0}; // maxVelocity, maxAcceleration, currentPosition, currentVelocity, targetPosition, targetVelocity

boolean testEncoderPID_value = false;
void testEncoderPID()
{
    serialLogln("Changing encoder PID setpoint!", 2);
    if (!testEncoderPID_value)
    {
        testEncoderPID_value = true;
        encoderATarget = encoderBTarget = TICKS_PER_ROTATION * 5;
    }
    else
    {
        testEncoderPID_value = false;
        encoderATarget = encoderBTarget = 0;
    }
}

float angle = 0;

void testTurn()
{
    serialLog("Changing destination angle to ", 2);
    serialLog(angle, 2);
    turn(M_PI / 180 * angle, "NULL");
    serialLog(" (", 2);
    serialLog(encoderATarget, 2);
    serialLog(", ", 2);
    serialLog(encoderBTarget, 2);
    serialLogln(")", 2);
}

bool firstEncoderVal = false;
bool secondEncoderVal = false;
uint8_t firstEncoderIndex = 0;
uint8_t secondEncoderIndex = 0;

unsigned long iteration = 0;
int maxTicks = 0;
bool movingXTicks = false;

//Here are the possible values:
    //0 = no encoder leading.
    //1 = left encoder leading.
    //2 = right encoder leading
uint8_t leadingEncoder = 0;
//previous encoder distance
float backPrevDistance = 0;

bool leftEncoderChange = false;
bool rightEncoderChange = false;

const uint8_t Top_Left_Encoder_Index = 0;
const uint8_t Top_Right_Encoder_Index = 1;
const uint8_t Bottom_Left_Encoder_Index = 2;
const uint8_t Bottom_Right_Encoder_Index = 3;

//determines basically what tile each encoder is one. false is one tile, true is the opposite tile.
//it could be false = white and true = black, or false = black and white = true, doesn't really matter
bool onFirstTile[4] = {false, false, false, false};

//put this in manually for each bot. Dist between the two front encoders, or the two back encoders. In meters.
const float lightDist = 0.07;

bool isCentering = true;
char centeringStatus = 'S';

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
        angle = 30;
        testTurn();
        timerInterval(5000, testTurn);
    }
}

// + (0.0001 * desiredVelocityA)
// Manages control loop (loopDelayMs is for reference)
void controlLoop(int loopDelayMs, int8_t framesUntilPrint) {
    currentEncoderA = readLeftEncoder();
    currentEncoderB = readRightEncoder();
    if (DO_LIGHT_SENSOR_TEST)
    {
        readLight(loopDelayMs);
        if(isCentering)
        {
            updateCentering();
        }
        
        // uint8_t status = driveUntilNewTile(onFirstTile);

        #if LOGGING_LEVEL >= 4
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
            serialLogln((char*) vals, 4);
            // switch(status)
            // {
            //     case 0:
            //         serialLogln((char*) "Driving not active!", 4);    
            //         break;
            //     case 1:
            //         serialLogln((char*) "Driving to tile!", 4);  
            //         break;
            //     case 2:
            //         serialLogln((char*) "We've reached the place!", 4); 
            //         break;
            //     case 3:
            //         serialLogln((char*) "Reversing backward!", 4); 
            //         break;
            //     case 4:
            //         serialLogln((char*) "Finished reversing back!", 4); 
            // }
        #endif
    }
    if (DO_ENCODER_TEST)
        encoderLoop();

    if (DO_PID) {
        // delay(50); // Delay to prevent CPU overload
        double loopDelaySeconds = ((double) loopDelayMs) / 1000;

        profileA.targetPosition = encoderATarget;
        profileA.currentPosition = currentEncoderA;

        profileB.targetPosition = encoderBTarget;
        profileB.currentPosition = currentEncoderB;
        
        double currentVelocityA = (currentEncoderA - prevPositionA) / loopDelaySeconds;
        profileA.currentVelocity = currentVelocityA;
        
        double currentVelocityB = (currentEncoderB - prevPositionB) / loopDelaySeconds;
        profileB.currentVelocity = currentVelocityB;
        
        double desiredVelocityA = updateTrapezoidalProfile(profileA, loopDelaySeconds, framesUntilPrint);
        double desiredVelocityB = updateTrapezoidalProfile(profileB, loopDelaySeconds, framesUntilPrint);
        
        prevPositionA = currentEncoderA;        
        prevPositionB = currentEncoderB;

        double leftFeedForward = desiredVelocityA / MAX_VELOCITY_TPS;
        double rightFeedForward = desiredVelocityB / MAX_VELOCITY_TPS;

        double leftMotorPower = encoderAVelocityController.Compute(desiredVelocityA, currentVelocityA, loopDelaySeconds) + leftFeedForward;
        double rightMotorPower = encoderBVelocityController.Compute(desiredVelocityB, currentVelocityB, loopDelaySeconds) + rightFeedForward;

        if (leftMotorPower > 1) leftMotorPower = 1;
        if (leftMotorPower < -1) leftMotorPower = -1;
        if (rightMotorPower > 1) rightMotorPower = 1;
        if (rightMotorPower < -1) rightMotorPower = -1;

        #if LOGGING_LEVEL >= 3
        
        if(framesUntilPrint == 0)
        {
            serialLog((char*) "Current encoder A pos: ", 2);
            serialLog(currentEncoderA, 2);
            serialLog(", ", 2);
            serialLog((char*) "Current encoder B pos: ", 2);
            serialLog(currentEncoderB, 2);
            serialLog(", ", 2);
            serialLog((char*) "Desired encoder A speed: ", 2);
            serialLog((float) desiredVelocityA, 2);
            serialLog(", ", 2);
            serialLog((char*) "Desired encoder B speed: ", 2);
            serialLog((float) desiredVelocityB, 2);
            serialLog(", ", 2);
            serialLog((char*) "current encoder a speed: ", 2);
            serialLog((float) currentVelocityA, 2);
            serialLog(", ", 2);
            serialLog((char*) "current encoder b speed: ", 2);
            serialLog((float) currentVelocityB, 2);
            serialLog(", ", 2);
            serialLog((char*) "current left motor power: ", 2);
            serialLog((float) leftMotorPower, 2);
            serialLog(", ", 2);
            serialLog((char*) "current right motor power: ", 2);
            serialLog((float) rightMotorPower, 2);
            serialLog(", ", 2);
            serialLog((char*) "current encoder a target: ", 2);
            serialLog(encoderATarget, 2);
            serialLog(", ", 2);
            serialLog((char*) "current encoder b target: ", 2);
            serialLog(encoderBTarget, 2); // TODO log results of trapezoidal profile into csv (on motor value graph)
            serialLog(", ", 2);
            serialLogln((float) loopDelaySeconds, 2);
        }
        
        #endif

        drive(
            leftMotorPower, // leftMotorPower,
            rightMotorPower, // rightMotorPower,
            "NULL"
        );
    }
}

void updateCentering()
{
    switch(centeringStatus)
    {
        //S means we just started centering
        case 'S':
            //create the first drive
            createDriveUntilNewTile(forwardAligning);
            //now change it so we're driving forward
            centeringStatus = 'M';
            break;
        //F means we are moving to an edge
        case 'M':
        {
            //continue driving forward
            uint8_t driveStatus = driveUntilNewTile();
            if(driveStatus == 3)
            {
                //meaning we can now go in opposite direction.
                centeringStatus = 'M';
                //if we were going forward, now reverse. if going reverse, now going forward
                forwardAligning = !forwardAligning;
                createDriveUntilNewTile(forwardAligning);
                serialLogln((char*) "NOW GOING IN OPPOSITE!", 2);
            }
            else if(driveStatus == 2)
            {
                //mean we now begin correcting
                centeringStatus = 'C';
                serialLogln((char*) "NOW CORRECTING!", 2);
                //we're going to check 
            }
            break;
        }
        case 'C':
            if(checkMoveFinished())
            {
                centeringStatus = 'M';
                forwardAligning = !forwardAligning;
                createDriveUntilNewTile(forwardAligning);
                serialLogln((char*) "NOW GOING IN OPPOSITE!", 2);
            }
            break;
    }
}

bool checkMoveFinished()
{
    //first check if we're past the encoder range, then see if our speed is low. Have to have the first critical range check as we don't want to count for
    //when we're first speeding up.
    float criticalRangeA = fabs(encoderATarget - startEncoderAPos) / 2;
    float criticalRangeB = fabs(encoderBTarget - startEncoderBPos) / 2;
    return (fabs(currentEncoderA - encoderATarget) < criticalRangeA && fabs(profileA.currentVelocity) < 2) && (fabs(currentEncoderB - encoderBTarget) < criticalRangeB  && fabs(profileB.currentVelocity) < 2);
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
void readLight(int loopDelayMs) {
    if(!waitingForLight)
    {
        waitingForLight = true;
        // The Infrared Blaster must be activated first to get a clear reading
        activateIR();
        //this way, we can pass in a parameter to timerDelay as well, but we don't have to
        timerDelay(loopDelayMs, std::bind(startLightReading, onFirstTile, &waitingForLight));
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

void updateToNextDistance(bool goingForward)
{
    serialLogln((char*) "changing direction!", 2);
    startEncoderAPos = currentEncoderA;
    startEncoderBPos = currentEncoderB;
    //this way if we're reversing, we're actually subtracting. if going forward was 0, then 0 * 2 - 1 = -1.
    //if going forward was 1, 1 * 2 - 1 = 1.
    encoderATarget = currentEncoderA + 1000 * (goingForward * 2 - 1);
    encoderBTarget = currentEncoderB + 1000 * (goingForward * 2 - 1);

    // if (!testEncoderPID_value)
    // {
    //     testEncoderPID_value = true;
    //     encoderATarget = currentEncoderA + 5000;
    //     encoderBTarget = currentEncoderB + 5000;
    // }
    // else
    // {
    //     testEncoderPID_value = false;
    //     encoderATarget = 350;
    //     encoderBTarget = 350;
    // }
    serialLogln(encoderATarget, 2);
    serialLogln(encoderBTarget, 2);
}

void createDriveUntilNewTile(bool goingForward)
{
    // drive(0.5f, 0.5f, "NULL");
    updateToNextDistance(goingForward);
    
    //assign values here, will detect when they change
    if(goingForward)
    {
        firstEncoderIndex = Top_Left_Encoder_Index;
        secondEncoderIndex = Top_Right_Encoder_Index;
    }
    else
    {
        firstEncoderIndex = Bottom_Left_Encoder_Index;
        secondEncoderIndex = Bottom_Right_Encoder_Index;
    }
    firstEncoderVal = onFirstTile[firstEncoderIndex];
    secondEncoderVal = onFirstTile[secondEncoderIndex];
}

//drives until the first two encoders are considered to hit a new value.
//Returns a status bit, with the correspnding values:
    // 1 - drive is going
    // 2 - just at this moment, we have reached our destination and are going to begin reversing now.
    // 3 - just at this moment, we've reached our destination, but DON'T need to reverse.
uint8_t driveUntilNewTile() 
{
    //if we do finish moving, update to a new distance
    if(checkMoveFinished())
    {
        updateToNextDistance(forwardAligning);
    }
    
    //if we already changed it, don't change it back again
    leftEncoderChange = leftEncoderChange || (onFirstTile[firstEncoderIndex] != firstEncoderVal);
    rightEncoderChange = rightEncoderChange || (onFirstTile[secondEncoderIndex] != secondEncoderVal);
    if(leftEncoderChange || rightEncoderChange)
    {
        //when both cross, we done.
        if(leftEncoderChange && rightEncoderChange)
        {
            //first get teh difference in encoders. Remember we want it from the encoder at the back.
            double backEncoderDist = (leadingEncoder == 1) ? fabs(currentEncoderB - backPrevDistance) : fabs(currentEncoderA - backPrevDistance);
#if LOGGING_LEVEL >= 3
            serialLog((char*) "End encoder is: ", 3);
            serialLogln((leadingEncoder == 1) ? currentEncoderA : currentEncoderB, 3);
            serialLog((char*) "Begin encoder is: ", 3);
            serialLogln(backPrevDistance, 3);
#endif
            //now convert difference in ticks to meter value

            backEncoderDist *= TICK_TO_METERS;
            // (fabs(frontCrossVelocity) + fabs(backCrossVelocity)) / 2 * backEncoderChangeInTime;
#if LOGGING_LEVEL >= 3
            serialLog((char*) " Encoder in front is gonna be: ", 2);
            serialLogln(leadingEncoder, 2);
            serialLog((char*) "Total distance encoder was behind is: ", 2);
            serialLogln((float) backEncoderDist, 2);
#endif
            //remidner: angle = arctan(x/y), where x = backEncoderDist (like distance to our edge), and y = distance between two light sensors (put in manually)

            //idk why, but when dividing by 2 that gets us the true angle. The BackEncoderDist and lightDist seems to be correct vales, but the value we
            //end up getting from it is double what it should be. Maybe there's an issue with how I did it.
            float radAngle = atan(backEncoderDist / lightDist) / 2;
            //as a reminder, corresponding degrees = (pi/180) * x radians

            float degreesAngle = 180 / M_PI * radAngle;
            
            serialLog((char*) "Angle is going to be: ", 2);
            serialLog(degreesAngle, 2);
            serialLogln((char*) " degrees.", 2);

            if(leadingEncoder != 0)
            {
                //we know postiive angle = turn left, negative angle means turn right.
                //encoder 1 forward means turn left, encoder 2 forward means turn right.
                //so 2 = -1, 1 = 1. If you plug those numbers in, that's what you get.

                //or at least it should be working that way, but for some reason it's not?
                //its going the exact opposite direction

                // int8_t degreesDirection = 3 - 2 * leadingEncoder;

                int8_t degreesDirection = 3 - 2 * leadingEncoder;
                serialLog((char*) "Sign is going to be: ", 2);
                serialLogln(degreesDirection, 2);
                angle = degreesAngle * degreesDirection;

                startEncoderAPos = currentEncoderA;
                startEncoderBPos = currentEncoderB;

                testTurn();
            }
            uint8_t status = (leadingEncoder == 0) ? 3 : 2; 

            leadingEncoder = 0;
            backEncoderDist = 0;
            backPrevDistance = 0;
            leftEncoderChange = false;
            rightEncoderChange = false;

            //if the encoder returned 0, no need to reverse so skip to status 3.
            return status;
        }
        //otherwise, want to see which one crossed. This is cond1 XOR cond2 btw.
        else if(backPrevDistance == 0)
        {
            //if the left one has crossed but the right hasn't, then just label that's the one crossing now.
            if(leftEncoderChange)
            {
                leadingEncoder = 1;
                backPrevDistance = currentEncoderB;
            }
            else
            {
                leadingEncoder = 2;
                backPrevDistance = currentEncoderA;
            }
            
        }
    }
    return 1;
}

// Tests the motors. This turns the motors off.
void driveTestOff() {
    stop();
    timerDelay(2000, &startDriveTest);
}

#endif