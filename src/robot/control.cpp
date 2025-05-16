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

MotionProfile profileA = {MAX_VELOCITY_TPS, MAX_ACCELERATION_TPSPS, 0, 0, 0, 0}; // maxVelocity, maxAcceleration, currentPosition, currentVelocity, targetPosition, targetVelocity
MotionProfile profileB = {MAX_VELOCITY_TPS, MAX_ACCELERATION_TPSPS, 0, 0, 0, 0}; // maxVelocity, maxAcceleration, currentPosition, currentVelocity, targetPosition, targetVelocity

//put this in manually for each bot. Dist between the two front encoders, or the two back encoders. In meters.
const float lightDist = 0.07;

//just measured, its 5.9 centimeters, or .059 meters
const float TIRE_RADIUS = 0.059;
//circumference equals pi * diameter. In meters
const float TIRE_CIRCUMFERENCE = M_PI * 2 * TIRE_RADIUS;
//multiply any ticks by this ratio to get distance in meters
const float TICK_TO_METERS = TIRE_CIRCUMFERENCE / TICKS_PER_ROTATION;

const uint8_t Top_Left_Encoder_Index = 0;
const uint8_t Top_Right_Encoder_Index = 1;
const uint8_t Bottom_Left_Encoder_Index = 2;
const uint8_t Bottom_Right_Encoder_Index = 3;

boolean testEncoderPID_value = false;

//determines the next target the encoders are going to
int encoderATarget = 0;
int encoderBTarget = 0;

//determines the encoder values the iteration right before
int prevPositionA = 0;
int prevPositionB = 0;

//gets the current value of the encoders
int currentEncoderA = -1;
int currentEncoderB = -1;

//when moving to a different target, this is the encoder position we started from
int startEncoderAPos = -1;
int startEncoderBPos = -1;

//the encoder distance to about halfway through the tile
int encoderAHalfwayDist = 0;
int encoderBHalfwayDist = 0;

//when we receive a command from the website to center, set this from false to true,
//then when the code sets it back to false, that's when we know we're done centering
bool isCentering = true;

//holds the current status representing the progress in centering. The differnet values
//are defined below
char centeringStatus = 'S';

//measures if the forward encoders are aligning on the edge or the back encoders
bool forwardAligning = true;

//set this true when our robot is on an edge and is moving to the center from its given measured distance
bool movingCenter = false;

//set this true when our robot is turning to align itself on the next axis
bool turningToNextAxis = false;

//initially 0, once 2 means both axises aligned so we done
uint8_t axisesAligned = 0;

float angle = 0;
unsigned long timeSinceTurn = 0;

//the value of the encoder we're measuring in terms of its light value
bool firstEncoderVal = false;
bool secondEncoderVal = false;

//represents which index is being referenced for the current encoders
uint8_t firstEncoderIndex = 0;
uint8_t secondEncoderIndex = 0;

//Here are the possible values:
    //0 = no encoder leading.
    //1 = left encoder leading.
    //2 = right encoder leading
uint8_t leadingEncoder = 0;

//previous encoder distance, speciically the one that is lagging behind
float backPrevDistance = 0;

//checks if the given encoders have changed tiles yet.
bool leftEncoderChange = false;
bool rightEncoderChange = false;

//critical range is explained on a different line
float criticalRangeA = 75;
float criticalRangeB = 75;

//determines basically what tile each encoder is one. false is one tile, true is the opposite tile.
//it could be false = white and true = black, or false = black and white = true, doesn't really matter
bool onFirstTile[4] = {false, false, false, false};

bool waitingForLight = false;

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

    updateCritRange();
}

//crit range is basically getting distance we go until we are halfway to target. By the end of this range,
//we're at our max speed and are sure we aren't at a low speed just cause we're speeding up
void updateCritRange()
{
    //want to record the values before we move now
    startEncoderAPos = currentEncoderA;
    startEncoderBPos = currentEncoderB;
    
    criticalRangeA = fabs(encoderATarget - startEncoderAPos) / 2;
    criticalRangeB = fabs(encoderBTarget - startEncoderBPos) / 2;

    //update it so that time since start is now equal to this. Only really care about this value after turns though
    timeSinceTurn = millis();
}

void testTurn()
{
    resetSpeed();
    
    serialLog("Changing destination angle to ", 2);
    serialLog(angle, 2);
    //simple maths
    turn(M_PI / 180 * angle, "NULL");
    serialLog(" (", 2);
    serialLog(encoderATarget, 2);
    serialLog(", ", 2);
    serialLog(encoderBTarget, 2);
    serialLogln(")", 2);

    //compute the new crit range now that target and start encoder have changed
    updateCritRange();
}

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
        //read the light values. May actually.... not want to do this if not centering? Keeping it outside the if
        //statement in case it'll be used for somethign else
        readLight(loopDelayMs);
        if(isCentering)
        {
            updateCentering();
        }
        
        // uint8_t status = driveUntilNewTile(onFirstTile);

        //this way we don't even upload the code if it's not being used
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
            serialLog((char*) "Light statuses: ", 2);
            serialLogln((char*) vals, 2);
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

        double desiredVelocityA = updateTrapezoidalProfile(profileA, loopDelaySeconds, framesUntilPrint, criticalRangeA);

        double desiredVelocityB = updateTrapezoidalProfile(profileB, loopDelaySeconds, framesUntilPrint, criticalRangeB);
        
        prevPositionA = currentEncoderA;        
        prevPositionB = currentEncoderB;

        double leftFeedForward = desiredVelocityA / MAX_VELOCITY_TPS;
        double rightFeedForward = desiredVelocityB / MAX_VELOCITY_TPS;

        double leftMotorPower = encoderAVelocityController.Compute(desiredVelocityA, currentVelocityA, loopDelaySeconds) + leftFeedForward;
        double rightMotorPower = encoderBVelocityController.Compute(desiredVelocityB, currentVelocityB, loopDelaySeconds) + rightFeedForward;

        
#if LOGGING_LEVEL >= 4
#endif

        if (leftMotorPower > 1) leftMotorPower = 1;
        if (leftMotorPower < -1) leftMotorPower = -1;
        if (rightMotorPower > 1) rightMotorPower = 1;
        if (rightMotorPower < -1) rightMotorPower = -1;

        //using macros this code isn't uploaded if not proper loging levels
        #if LOGGING_LEVEL >= 4
        
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

//this determines what our next action will be after an edge alignment, move to center, or axis turn.
void determineNextAction()
{
    //if we were previously turning to the next axis, and finished:
    if(turningToNextAxis)
    {
        turningToNextAxis = false;
        axisesAligned++;
        //done centering yippee!
        if(axisesAligned == 2)
        {
            axisesAligned = 0;
            isCentering = false;
            centeringStatus = 'F';
        }
        //otherwise restart the whole process for this next axis
        else
        {
            centeringStatus = 'S';
        }
    }
    //if preivously moving to center for an axis, and finished:
    else if(movingCenter)
    {
        //now, turn to next direction
        movingCenter = false;
        turningToNextAxis = true;
        centeringStatus = 'M';
        //this way, if no axises aligned yet, angle = -90, and if one axis aligned,
        //angle = 90, undoing the previous rotation.
        angle = 90 * (axisesAligned * 2 - 1);

        testTurn();
    }
    //if just aligned on a forward edge with the encoders in the front:
    else if(forwardAligning)
    {
        //if going forward, store the current encoder value so we can see the full encoder length of the tiles
        encoderAHalfwayDist = currentEncoderA;
        encoderBHalfwayDist = currentEncoderB;
        //swap to going back
        forwardAligning = !forwardAligning;
        //set that new drive
        createDriveUntilNewTile();
        centeringStatus = 'E';

        serialLog((char*)"Current encoder A: ", 2);
        serialLogln(currentEncoderA, 2);
        serialLog((char*)"Current encoder B: ", 2);
        serialLogln(currentEncoderB, 2);
        serialLog((char*)"Target encoder A: ", 2);
        serialLogln(encoderATarget, 2);
        serialLog((char*)"Target encoder B: ", 2);
        serialLogln(encoderBTarget, 2);
        
    }
    //if just aligned on a backward edge with the encoders in the back:
    else
    {
        //swap to going forward now
        forwardAligning = !forwardAligning;
        
        //since we always go forward first and then backwards, the current value of encoderHalfwayDist > currentEncoder always
        //what we're doing is currently, "encoderAHalfwayDist" just stores the value of the other edge in encoder ticks, now we're
        //finding the difference between them. And of course divide by 2 as want half that distance
        encoderAHalfwayDist = (encoderAHalfwayDist - currentEncoderA) / 2;
        encoderBHalfwayDist = (encoderBHalfwayDist - currentEncoderB) / 2;

        //decide that we take average of encoder A and B's distances, since ideally we want both to travel the same amount
        int totalHalfwayDistance = (encoderAHalfwayDist + encoderBHalfwayDist) / 2;
        driveTicks(totalHalfwayDistance, "NULL");
        centeringStatus = 'M';
        //now we moving to da center
        movingCenter = true;
    }
}

//this updates our centering depending on the current statis of it
void updateCentering()
{
    //what we do depend on the current status
    switch(centeringStatus)
    {
        //What each status bit means:
        //  S means we just started centering on the current axis
        //  E means we are moving to an edge
        //  M means the bot is just moving by itself, whether that be to correct for a turn or to just get to the halfway point
        case 'S':
            //create the first drive
            createDriveUntilNewTile();
            //now change it so we're driving forward
            centeringStatus = 'E';
            break;
        case 'E':
        {
            //continue driving forward
            uint8_t driveStatus = driveUntilNewTile();
            //reminder status 3 = no leading encodere and driving finished
            if(driveStatus == 3)
            {
                //meaning we can now go in opposite direction.
                determineNextAction();
            }
            //reminder status 2 = one leading encoder finished first, but driving is finished
            else if(driveStatus == 2)
            {
                //mean we now begin correcting
                centeringStatus = 'M';
            }
            break;
        }
        case 'M':
            //check if we're the moving we're doing is finished. If so, determine what we do next.
            if(checkMoveFinished())
            {
                determineNextAction();
            }
            break;
    }
}

//checks if we're done moving to our target when either moving half a tile's length or turning
bool checkMoveFinished()
{
    //first, checks like "fabs(profileA.currentVelocity) < 2" see if we're slowing down or not.

    //then, something like "fabs(currentEncoderA - encoderATarget) < criticalRangeA" is making sure
    //the reason our speed is slow is specifically because we're slowing down and not because we're
    //beginning to speed up.
    //do this by seeing if the distance remaining is less than the midpoint distance from start to end, as by then we're at our max speed.

    bool encoderAChecks = fabs(currentEncoderA - encoderATarget) < criticalRangeA && fabs(profileA.currentVelocity) < 3;
    bool encoderBChecks = fabs(currentEncoderB - encoderBTarget) < criticalRangeB  && fabs(profileB.currentVelocity) < 3;

    //check if we've been stalling too long, for 8 seconds. If we're over time, that's bad, annd means we should declare the movement finished.
    bool timerCheck = millis() - timeSinceTurn > 8000;
    
    return ((encoderAChecks && encoderBChecks) || timerCheck);
}

//like the one above but just seeing if we can keep moving or not
bool checkIfCanUpdateMovement()
{
    return fabs(currentEncoderA - encoderATarget) < criticalRangeA && fabs(currentEncoderB - encoderBTarget) < criticalRangeB;
}

// Drives a specific amount of tiles (WIP)
void drive(float tiles) {

}

//drives the given amount of ticks
void driveTicks(int tickDistance, std::string id)
{    
    resetSpeed();
    encoderATarget = currentEncoderA + tickDistance;
    encoderBTarget = currentEncoderB + tickDistance;

    updateCritRange();
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

//turns the given amount in radians
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

// Reads in the light value of all light sensors
void readLight(int loopDelayMs) {
    //if we're still waiting to read, can't enter this statement. Kind of like a mutex lock
    if(!waitingForLight)
    {
        waitingForLight = true;
        // The Infrared Blaster must be activated first to get a clear reading
        activateIR();
        //this way, we can pass in a parameter to timerDelay as well, but we don't have to
        timerDelay(loopDelayMs, std::bind(startLightReading, onFirstTile, &waitingForLight));
    }
}

void resetSpeed()
{
    profileA.targetVelocity = 0;
    profileB.targetVelocity = 0;
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

//updates us to the next distance we're traveling
void updateToNextDistance()
{
    //this way if we're reversing, we're actually subtracting. if going forward was 0, then 0 * 2 - 1 = -1.
    //if going forward was 1, 1 * 2 - 1 = 1.
    encoderATarget = currentEncoderA + 2500 * (forwardAligning * 2 - 1);
    encoderBTarget = currentEncoderB + 2500 * (forwardAligning * 2 - 1);
#if LOGGING_LEVEL >= 3
    serialLogln((char*) "changing direction!", 3);
    serialLogln(encoderATarget, 3);
    serialLogln(encoderBTarget, 3);
#endif

    //update crit range yessir
    updateCritRange();
}

//creates a new drive until the next tile
void createDriveUntilNewTile()
{
    //set the next distance to travel
    resetSpeed();
    updateToNextDistance();
    
    //assign values here, will detect when they change
    if(forwardAligning)
    {
        firstEncoderIndex = Top_Left_Encoder_Index;
        secondEncoderIndex = Top_Right_Encoder_Index;
    }
    else
    {
        //from the robots perspective as it's going backwards, the encoder on the left is bottom right, 
        //while the encoder on the right is bottom left, so we just swap em
        firstEncoderIndex = Bottom_Right_Encoder_Index;
        secondEncoderIndex = Bottom_Left_Encoder_Index;
    }
    //now set the actual values with the indices
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
    //if we do finish moving, update to a new distance. I think we have to move at intervals for it to work
    if(checkIfCanUpdateMovement())
    {
        updateToNextDistance();
    }
    
    //if we already changed it, don't change it back again
    leftEncoderChange = leftEncoderChange || (onFirstTile[firstEncoderIndex] != firstEncoderVal);
    rightEncoderChange = rightEncoderChange || (onFirstTile[secondEncoderIndex] != secondEncoderVal);

    //if either has changed
    if(leftEncoderChange || rightEncoderChange)
    {
        //when both cross, we done.
        if(leftEncoderChange && rightEncoderChange)
        {
            //if there had been a leading encoder, do the following:
            uint8_t status = 0; 
            if(leadingEncoder != 0)
            {
                encoderATarget = currentEncoderA;
                encoderBTarget = currentEncoderB;
                
                //first get the encoder that we're comparing to get the distance. If leading encoder is 1 and we were moving forward, or if
                //leading encoder is 2 but we were moving backward, then the back encoder is b, otherwise it's A
                double encoderChosen = (leadingEncoder == 1 && forwardAligning || leadingEncoder == 2 && !forwardAligning) ? currentEncoderB : currentEncoderA;
                //now, the idstance we traveled is the difference between the encoders
                double backEncoderDist = fabs(encoderChosen - backPrevDistance);
#if LOGGING_LEVEL >= 3
                serialLog((char*) "Begin encoder is: ", 2);
                serialLogln(backPrevDistance, 2);
                serialLog((char*) "End encoder is: ", 2);
                serialLogln((leadingEncoder == 1) ? currentEncoderA : currentEncoderB, 2);
#endif
                //now convert difference in ticks to meter value
                backEncoderDist *= TICK_TO_METERS;
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
                
                //we know postiive angle = turn left, negative angle means turn right.
                //encoder 1 forward means turn left, encoder 2 forward means turn right.
                //so 2 = -1, 1 = 1. If you plug those numbers in, that's what you get.

                int8_t degreesDirection = 3 - 2 * leadingEncoder;
                serialLog((char*) "Sign is going to be: ", 2);
                serialLogln(degreesDirection, 2);

                //now update the angle, and turn
                angle = degreesAngle * degreesDirection;
                testTurn();
                status = 2;
            }
            else
            {
                status = 3;

                //with the other one we set the crit range after calling the turn function.
                //Here we don't, so we gotta do that
                updateCritRange();
            }

            //reset all of our values
            leadingEncoder = 0;
            backPrevDistance = 0;
            leftEncoderChange = false;
            rightEncoderChange = false;

            //if the encoder returned 0, no need to reverse so skip to status 3.
            return status;
        }
        //otherwise, want to see which one crossed. Can do this by seeing if we've updated backPrevDistance yet or not.
        else if(backPrevDistance == 0)
        {
            //if left encoder changed, that's the leading encoder, otherwise 2nd is the leading encoder obviously.
            leadingEncoder = leftEncoderChange ? 1 : 2;
            //this makes sense as if the left encoder first crossed but we're going backwards, that's encoder A that's behind. Meanwhile if the right
            //encoder crossed first and we're going forwards, its encoder A that's behind too. In all other cases, its encoder B
            backPrevDistance = (leftEncoderChange && !forwardAligning || rightEncoderChange && forwardAligning) ? currentEncoderA : currentEncoderB;            
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