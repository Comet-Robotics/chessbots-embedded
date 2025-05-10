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

int angle = 0;

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

unsigned long iteration = 0;
int maxTicks = 0;
bool movingXTicks = false;

//Here are the possible values:
    //0 = no encoder leading.
    //1 = left encoder leading.
    //2 = right encoder leading
uint8_t leadingEncoder = 0;
//Ticks it tackes for back encoder to reach tile change after the front tile does.
unsigned long backEncoderDist = 0;
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
char centeringStatus = 'Z';

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
void controlLoop(int loopDelayMs) {
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
            switch(status)
            {
                case 0:
                    serialLogln((char*) "Driving not active!", 4);    
                    break;
                case 1:
                    serialLogln((char*) "Driving to tile!", 4);  
                    break;
                case 2:
                    serialLogln((char*) "We've reached the place!", 4); 
                    break;
                case 3:
                    serialLogln((char*) "Reversing backward!", 4); 
                    break;
                case 4:
                    serialLogln((char*) "Finished reversing back!", 4); 
            }
        #endif
    }
    if (DO_ENCODER_TEST)
        encoderLoop();

    if (DO_PID) {
        // delay(50); // Delay to prevent CPU overload
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
        if (leftMotorPower < -1) leftMotorPower = 1;
        if (rightMotorPower > 1) rightMotorPower = 1;
        if (rightMotorPower < -1) rightMotorPower = 1;

        serialLog((char*) "Current encoder A pos: ", 3);
        serialLog(currentPositionEncoderA, 3);
        serialLog(", ", 3);
        serialLog((char*) "Current encoder B pos: ", 3);
        serialLog(currentPositionEncoderB, 3);
        serialLog(", ", 3);
        serialLog((char*) "Desired encoder A speed: ", 3);
        serialLog((float) desiredVelocityA, 3);
        serialLog(", ", 3);
        serialLog((char*) "Desired encoder B speed: ", 3);
        serialLog((float) desiredVelocityB, 3);
        serialLog(", ", 3);
        serialLog((char*) "current encoder a speed: ", 3);
        serialLog((float) currentVelocityA, 3);
        serialLog(", ", 3);
        serialLog((char*) "current encoder b speed: ", 3);
        serialLog((float) currentVelocityB, 3);
        serialLog(", ", 3);
        serialLog((char*) "current left motor power: ", 3);
        serialLog((float) leftMotorPower, 3);
        serialLog(", ", 3);
        serialLog((char*) "current right motor power: ", 3);
        serialLog((float) rightMotorPower, 3);
        serialLog(", ", 3);
        serialLog((char*) "current encoder a target: ", 3);
        serialLog(encoderATarget, 3);
        serialLog(", ", 3);
        serialLog((char*) "current encoder b target: ", 3);
        serialLog(encoderBTarget, 3); // TODO log results of trapezoidal profile into csv (on motor value graph)
        serialLog(", ", 3);
        serialLogln((float) loopDelaySeconds, 3);

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
            createDriveUntilNewTile();
            //now change it so we're driving forward
            centeringStatus = 'F';
            break;
        //F means we are driving forward
        case 'F':
        {
            //continue driving forward
            uint8_t driveStatus = driveUntilNewTile();
            if(driveStatus == 3)
            {
                //meaning we can now go in reverse
                centeringStatus = 'R';
            }
            else if(driveStatus == 2)
            {
                //mean we now begin correcting
                centeringStatus = 'C';
            }
            break;
        }
        case 'C':
            break;
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
void readLight(int loopDelayMs) {
    if(!waitingForLight)
    {
        // The Infrared Blaster must be activated first to get a clear reading
        activateIR();
        waitingForLight = true;
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

void createDriveUntilNewTile()
{
    drive(0.5f, 0.5f, "NULL");
    //assign values here, will detect when they change
    firstEncoderVal = onFirstTile[Top_Left_Encoder_Index];
    secondEncoderVal = onFirstTile[Top_Right_Encoder_Index];
}

//drives until the first two encoders are considered to hit a new value.
//Returns a status bit, with the correspnding values:
    // 1 - drive is going
    // 2 - just at this moment, we have reached our destination and are going to begin reversing now.
    // 3 - just at this moment, we've reached our destination, but DON'T need to reverse.
uint8_t driveUntilNewTile() 
{
    //if we already changed it, don't change it back again
    leftEncoderChange = leftEncoderChange || (onFirstTile[Top_Left_Encoder_Index] != firstEncoderVal);
    rightEncoderChange = rightEncoderChange || (onFirstTile[Top_Right_Encoder_Index] != secondEncoderVal);
    if(leftEncoderChange || rightEncoderChange)
    {
        //when both cross, we done.
        if(leftEncoderChange && rightEncoderChange)
        {
            //subtract end time by start time, backEncoderDist stores start time.
            backEncoderDist = (leadingEncoder != 0) ? millis() - backEncoderDist : 0;

            stop();
            serialLog((char*) " Encoder in front is gonna be: ", 2);
            serialLogln(leadingEncoder, 2);
            serialLog((char*) "And distance back encoder was behind is: ", 2);
            serialLogln((int) backEncoderDist, 2);

            float tickCountToDistMultiplier = 0.000222;
            //remidner: angle = arctan(x/y), where x = backEncoderDist * tickCounToDistMultiplier (like distance to our edge), and y = distance between two light sensors (put in manually)
            float angle = atan(backEncoderDist * tickCountToDistMultiplier / lightDist);
            //as a reminder, corresponding degrees = (pi/180) * x radians
            float degreesAngle = 180 / M_PI * angle;
            
            serialLog((char*) "Angle is going to be: ", 2);
            serialLog(degreesAngle, 2);
            serialLogln((char*) " degrees.", 2);

            if(leadingEncoder != 0)
            {
                turn(angle, "NULL");
            }

            backEncoderDist = 0;
            leadingEncoder = 0;
            //if the encoder returned 0, no need to reverse so skip to status 3.
            return (leadingEncoder == 0) ? 3 : 2;    
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
            //set it to the current ms time, we weill subtract it by the ms time when we end to get the duration it takes.
            backEncoderDist = millis();
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