#ifndef CHESSBOT_CONTROL_CPP
#define CHESSBOT_CONTROL_CPP

// Associated Header File
#include "robot/control.h"

// Built-In Libraries
#include "Arduino.h"
#include <queue>

// Custom Libraries
#include "utils/logging.h"
#include "utils/timer.h"
#include "utils/config.h"
#include "utils/status.h"
#include "robot/motor.h"
#include "robot/lightSensor.h"
#include "wifi/connection.h"
#include "robot/encoder.h"
#include "robot/pidController.h"
#include "robot/driveTest.h"
#include "../../env.h"
#include <algorithm>
#include "utils/functions.h"

#include "robot/profiledPIDController.h"
#include "robot/trapezoidalProfileNew.h"

//PLEASE ONLY USE CHESSBOT #4 FOR TESTING
TrapezoidProfile::Constraints profileConstraints(MAX_VELOCITY_TPS, MAX_ACCELERATION_TPSPS);
TrapezoidProfile leftProfile(profileConstraints);
TrapezoidProfile rightProfile(profileConstraints);
TrapezoidProfile::State leftSetpoint, rightSetpoint;
PIDController encoderAVelocityController(0.00004, 0.000005, 0.00000, -1, +1); // Blue
PIDController encoderBVelocityController(0.00004, 0.000005, 0.00000, -1, +1); //Red

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

//determines the encoder values the iteration right before
int prevPositionA = 0;
int prevPositionB = 0;

//when moving to a different target, this is the encoder position we started from
int startEncoderAPos = -1;
int startEncoderBPos = -1;

//the encoder distance to about halfway through the tile
int encoderAHalfwayDist = 0;
int encoderBHalfwayDist = 0;

//when we receive a command from the website to center, set this from false to true,
//then when the code sets it back to false, that's when we know we're done centering
bool isCentering = false;

//holds the current status representing the progress in centering. The different values
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

//previous encoder distance, specifically the one that is lagging behind
float backPrevDistance = 0;

//checks if the given encoders have changed tiles yet.
bool leftEncoderChange = false;
bool rightEncoderChange = false;

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
        setLeftMotorControl({POSITION, (float)TICKS_PER_ROTATION * 5});
        setRightMotorControl({POSITION, (float)TICKS_PER_ROTATION * 5});
    }
    else
    {
        testEncoderPID_value = false;
        setLeftMotorControl({POSITION, 0.0f});
        setRightMotorControl({POSITION, 0.0f});
    }

    updateCritRange();
}

//crit range is basically getting distance we go until we are halfway to target. By the end of this range,
//we're at our max speed and are sure we aren't at a low speed just cause we're speeding up
void updateCritRange()
{
    //want to record the values before we move now
    startEncoderAPos = profileA.currentPosition;
    startEncoderBPos = profileB.currentPosition;
    
    profileA.criticalRange = fabs(profileA.targetPosition - startEncoderAPos) / 2;
    profileB.criticalRange = fabs(profileB.targetPosition - startEncoderBPos) / 2;

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
    serialLog(getLeftMotorControl().value, 2);
    serialLog(", ", 2);
    serialLog(getRightMotorControl().value, 2);
    serialLogln(")", 2);

    //compute the new crit range now that target and start encoder have changed
    updateCritRange();
}

void testCentering()
{
    serialLogln("Running centering routine...", 2);
    startCentering();
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
        timerInterval(12000, &testEncoderPID);
    }
    
    if (DO_TURN_TEST) {
        angle = 30;
        testTurn();
        timerInterval(5000, testTurn);
    }

    if (DO_CENTERING_TEST) {
        testCentering();
        timerInterval(5000, &testCentering);
    }
}

// + (0.0001 * desiredVelocityA)
// Manages control loop (loopDelayMs is for reference)
void controlLoop(int loopDelayMs, int8_t framesUntilPrint) {
    if (DO_LIGHT_SENSOR_TEST)
    {
        //read the light values. May actually.... not want to do this if not centering? Keeping it outside the if
        //statement in case it'll be used for something else
        readLight(loopDelayMs);
        if(isCentering)
        {
            // TODO see if targetPosition needs to be updated in the middle of this method - doesn't look like it?
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
            serialLog("Light statuses: ", 2);
            serialLogln(vals, 2);
        #endif
    }

    if (DO_ENCODER_TEST) encoderLoop();

    if (DO_PID) {
        double loopDelaySeconds = ((double) loopDelayMs) / 1000;

        int currentPositionEncoderA = readLeftEncoder();
        int currentPositionEncoderB = readRightEncoder();
        double currentVelocityA = (currentPositionEncoderA - prevPositionA) / loopDelaySeconds;
        double currentVelocityB = (currentPositionEncoderB - prevPositionB) / loopDelaySeconds;

        profileA.currentPosition = currentPositionEncoderA;
        profileA.currentVelocity = currentVelocityA;
        profileB.currentPosition = currentPositionEncoderB;
        profileB.currentVelocity = currentVelocityB;

        // Generate trapezoidal profile setpoints
        if (getLeftMotorControl().mode == POSITION) {
            leftSetpoint = leftProfile.calculate(loopDelaySeconds, 
                                                leftSetpoint,
                                                TrapezoidProfile::State(getLeftMotorControl().value, 0.0));
        } else {
            leftSetpoint = TrapezoidProfile::State(currentPositionEncoderA, getLeftMotorControl().value);
        }
        if (getRightMotorControl().mode == POSITION) {
            rightSetpoint = rightProfile.calculate(loopDelaySeconds, 
                                                rightSetpoint,
                                                TrapezoidProfile::State(getRightMotorControl().value, 0.0));
        } else {
            rightSetpoint = TrapezoidProfile::State(currentPositionEncoderB, getRightMotorControl().value);
        }

        prevPositionA = currentPositionEncoderA;
        prevPositionB = currentPositionEncoderB;

        double leftFeedForward = leftSetpoint.velocity / MAX_VELOCITY_TPS;
        double rightFeedForward = rightSetpoint.velocity / MAX_VELOCITY_TPS;

        double leftMotorPower = encoderAVelocityController.Compute(leftSetpoint.velocity, currentVelocityA, loopDelaySeconds) + leftFeedForward;
        double rightMotorPower = encoderBVelocityController.Compute(rightSetpoint.velocity, currentVelocityB, loopDelaySeconds) + rightFeedForward;

        if (leftMotorPower > 1) leftMotorPower = 1;
        if (leftMotorPower < -1) leftMotorPower = -1;
        if (rightMotorPower > 1) rightMotorPower = 1;
        if (rightMotorPower < -1) rightMotorPower = -1;

        //using macros this code isn't uploaded if not proper loging levels
        #if LOGGING_LEVEL >= 3
        
        if(framesUntilPrint == 0)
        {
            // serialLog("Current encoder A pos: ", 2);
            // serialLog(currentPositionEncoderA, 2);
            // serialLog(", ", 2);
            // serialLog("Current encoder B pos: ", 2);
            // serialLog(currentPositionEncoderB, 2);
            // serialLog(", ", 2);
            // serialLog("Desired encoder A speed: ", 2);
            // serialLog(leftSetpoint.velocity, 2);
            // serialLog(", ", 2);
            // serialLog("Desired encoder B speed: ", 2);
            // serialLog(rightSetpoint.velocity, 2);
            // serialLog(", ", 2);
            // serialLog("current encoder a speed: ", 2);
            // serialLog(currentVelocityA, 2);
            // serialLog(", ", 2);
            // serialLog("current encoder b speed: ", 2);
            // serialLog(currentVelocityB, 2);
            // serialLog(", ", 2);
            // serialLog("current left motor power: ", 2);
            // serialLog(leftMotorPower, 2);
            // serialLog(", ", 2);
            // serialLog("current right motor power: ", 2);
            // serialLog(rightMotorPower, 2);
            // serialLog(", ", 2);
            // serialLog("current encoder a target: ", 2);
            // serialLog(leftMotorControl.mode == POSITION ? leftMotorControl.value : 0, 2);
            // serialLog(", ", 2);
            // serialLog("current encoder b target: ", 2);
            // serialLog(rightMotorControl.mode == POSITION ? rightMotorControl.value : 0, 2); // TODO log results of trapezoidal profile into csv (on motor value graph)
            // serialLog(", ", 2);
            // serialLog("is robot pid at target? ", 2);
            // serialLog(isRobotPidAtTarget(), 2);
            // serialLog(", ", 2);
            // serialLogln(loopDelaySeconds, 2);
        }
    
        serialLog(leftSetpoint.velocity, 3);
        serialLog(",", 3);
        serialLog(currentVelocityA, 3);
        serialLog(",", 3);
        serialLog(leftSetpoint.velocity - currentVelocityA, 3);
        serialLog(",", 3);
        serialLog(rightSetpoint.velocity, 3);
        serialLog(",", 3);
        serialLog(currentVelocityB, 3);
        serialLog(",", 3);
        serialLogln(rightSetpoint.velocity - currentVelocityB, 3);

        #endif

        drive(
            leftMotorPower, // leftMotorPower,
            rightMotorPower, // rightMotorPower,
            "NULL"
        );

        // serialLogln(leftMotorPower, 3);

        // turn(M_PI / 2, "NULL");
    }
}

// start centering process
void startCentering()
{
    if (!isCentering)
    {
        isCentering = true;
        centeringStatus = 'S';
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
    //if previously moving to center for an axis, and finished:
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
        encoderAHalfwayDist = profileA.currentPosition;
        encoderBHalfwayDist = profileB.currentPosition;
        //swap to going back
        forwardAligning = !forwardAligning;
        //set that new drive
        createDriveUntilNewTile();
        centeringStatus = 'E';

        serialLog((char*)"Current encoder A: ", 2);
        serialLogln(profileA.currentPosition, 2);
        serialLog((char*)"Current encoder B: ", 2);
        serialLogln(profileB.currentPosition, 2);
        serialLog((char*)"Target encoder A: ", 2);
        serialLogln(leftMotorControl.mode == POSITION ? leftMotorControl.value : 0, 2);
        serialLog((char*)"Target encoder B: ", 2);
        serialLogln(rightMotorControl.mode == POSITION ? rightMotorControl.value : 0, 2);
        }
    //if just aligned on a backward edge with the encoders in the back:
    else
    {
        //swap to going forward now
        forwardAligning = !forwardAligning;
        
        //since we always go forward first and then backwards, the current value of encoderHalfwayDist > currentEncoder always
        //what we're doing is currently, "encoderAHalfwayDist" just stores the value of the other edge in encoder ticks, now we're
        //finding the difference between them. And of course divide by 2 as want half that distance
        encoderAHalfwayDist = (encoderAHalfwayDist - profileA.currentPosition) / 2;
        encoderBHalfwayDist = (encoderBHalfwayDist - profileB.currentPosition) / 2;

        //decide that we take average of encoder A and B's distances, since ideally we want both to travel the same amount
        int totalHalfwayDistance = (encoderAHalfwayDist + encoderBHalfwayDist) / 2;
        driveTicks(totalHalfwayDistance, "NULL");
        centeringStatus = 'M';
        //now we moving to da center
        movingCenter = true;
    }
}

//this updates our centering depending on the current status of it
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
            //reminder status 3 = no leading encoders and driving finished
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

    bool encoderAChecks = fabs(profileA.currentPosition - profileA.targetPosition) < profileA.criticalRange && fabs(profileA.currentVelocity) < 3;
    bool encoderBChecks = fabs(profileB.currentPosition - profileB.targetPosition) < profileB.criticalRange  && fabs(profileB.currentVelocity) < 3;

    //check if we've been stalling too long, for 8 seconds. If we're over time, that's bad, and means we should declare the movement finished.
    bool timerCheck = millis() - timeSinceTurn > 8000;
    
    return ((encoderAChecks && encoderBChecks) || timerCheck);
}

//like the one above but just seeing if we can keep moving or not
bool checkIfCanUpdateMovement()
{
    return fabs(profileA.currentPosition - profileA.targetPosition) < profileA.criticalRange && fabs(profileB.currentPosition - profileB.targetPosition) < profileB.criticalRange;
}

void setLeftMotorControl(ControlSetting control) {
    leftMotorControl = control;
    leftSetpoint = TrapezoidProfile::State(readLeftEncoder(), profileA.currentVelocity);
    if (control.mode == POSITION)
        profileA.targetPosition = control.value;
    else
        profileA.targetVelocity = control.value;
}

void setRightMotorControl(ControlSetting control) {
    rightMotorControl = control;
    rightSetpoint = TrapezoidProfile::State(readRightEncoder(), profileB.currentVelocity);
    if (control.mode == POSITION)
        profileB.targetPosition = control.value;
    else
        profileB.targetVelocity = control.value;
}

ControlSetting getLeftMotorControl() {
    return leftMotorControl;
}

ControlSetting getRightMotorControl() {
    return rightMotorControl;
}

void drive(float tiles, std::string id) {
    if (!getStoppedStatus()) {
        const float TILE_SIZE_INCHES = 24;
        float distanceInches = tiles * TILE_SIZE_INCHES;
        float ticksPerInch = TICKS_PER_ROTATION / (WHEEL_DIAMETER_INCHES * M_PI);
        int tickDistance = (int)(distanceInches * ticksPerInch);
        driveTicks(tickDistance, id);
    }    
    
}

//drives the given amount of ticks
void driveTicks(int tickDistance, std::string id)
{
    if (!getStoppedStatus()) {
        resetSpeed();
        setLeftMotorControl({POSITION, (float)(readLeftEncoder() + tickDistance)});
        setRightMotorControl({POSITION, (float)(readRightEncoder() + tickDistance)});
        updateCritRange();

        if (id != "NULL") {
            sendPacketOnPidComplete(id);
        }
    }
}

// Drives the wheels according to the powers set. Negative is backwards, Positive forwards
void drive(float leftPower, float rightPower, std::string id) {
    if (!getStoppedStatus()) {
        // TODO: maybe move to motor.cpp?
        float minPower = 0.16;
        if (leftPower < minPower && leftPower > -minPower) {
            leftPower = 0;
        } if (rightPower < minPower && rightPower > -minPower) {
            rightPower = 0;
        }

        setLeftPower(leftPower);
        setRightPower(rightPower);

        //we only send null as id during our test drive. The only other time this drive method is called will be
        //when the server sends it, meaning it will have an id to send back.
        if (id != "NULL") { sendActionSuccess(id); }
    }
}

//turns the given amount in radians
void turn(float angleRadians, std::string id) {

    serialLogln("Turning", 3);
    serialLogln(angleRadians, 3);
    int offsetTicks = radiansToTicks(angleRadians);

    if (getLeftMotorControl().mode == POSITION) {
        setLeftMotorControl({POSITION, getLeftMotorControl().value - offsetTicks});
    } else {
        setLeftMotorControl({POSITION, (float)(readLeftEncoder() - offsetTicks)});
    }
    if (getRightMotorControl().mode == POSITION) {
        setRightMotorControl({POSITION, getRightMotorControl().value + offsetTicks});
    } else {
        setRightMotorControl({POSITION, (float)(readRightEncoder() + offsetTicks)});
    }

    if (id != "NULL")
    {
        sendPacketOnPidComplete(id);
    }
}

// Stops the bot in its tracks
void stop() {
    setStoppedStatus(true);
    setLeftPower(0);
    setRightPower(0);

    serialLogln("Bot Stopped!", 2);
}

boolean isRobotPidAtTarget() {
    if (!DO_PID)
        return true;

    boolean leftAtTarget, rightAtTarget;

    if (getLeftMotorControl().mode == POSITION)
    {
        leftAtTarget = approxEquals(getLeftMotorControl().value, profileA.currentPosition, PID_POSITION_TOLERANCE)
                    && approxEquals(profileA.currentVelocity, 0.0, PID_VELOCITY_TOLERANCE);

        serialLog("Left Position: ", 3);
        serialLog(profileA.currentPosition, 3);
        serialLog(", Target: ", 3);
        serialLog(getLeftMotorControl().value, 3);
        serialLog(", Velocity: ", 3);
        serialLog(profileA.currentVelocity, 3);
        serialLog(", At Target: ", 3);
        serialLogln(leftAtTarget, 3);
    }
    else
    {
        leftAtTarget = approxEquals(getLeftMotorControl().value, profileA.currentVelocity, PID_VELOCITY_TOLERANCE);
    }
    if (getRightMotorControl().mode == POSITION)
    {
        rightAtTarget = approxEquals(getRightMotorControl().value, profileB.currentPosition, PID_POSITION_TOLERANCE)
                     && approxEquals(profileB.currentVelocity, 0.0, PID_VELOCITY_TOLERANCE);

        serialLog("Right Position: ", 3);
        serialLog(profileB.currentPosition, 3);
        serialLog(", Target: ", 3);
        serialLog(getRightMotorControl().value, 3);
        serialLog(", Velocity: ", 3);
        serialLog(profileB.currentVelocity, 3);
        serialLog(", At Target: ", 3);
        serialLogln(rightAtTarget, 3);
    }
    else
    {
        rightAtTarget = approxEquals(getRightMotorControl().value, profileB.currentVelocity, PID_VELOCITY_TOLERANCE);
    }

    return leftAtTarget && rightAtTarget;
}

void sendPacketOnPidComplete(std::string id) {
    if (!DO_PID)
        sendActionFail(id);

    if (isRobotPidAtTarget()) {
        sendActionSuccess(id);
    } else {
        // Run on next loop
        timerDelay(1, [id](){ sendPacketOnPidComplete(id); });
    }
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
    float encTargetA = (float)profileA.currentPosition + 2500 * (forwardAligning * 2 - 1);
    float encTargetB = (float)profileB.currentPosition + 2500 * (forwardAligning * 2 - 1);
    setLeftMotorControl({POSITION, encTargetA});
    setRightMotorControl({POSITION, encTargetB});
#if LOGGING_LEVEL >= 3
    serialLogln("changing direction!", 3);
    serialLogln(encTargetA, 3);
    serialLogln(encTargetB, 3);
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
//Returns a status bit, with the corresponding values:
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
                setLeftMotorControl({POSITION, (float)profileA.currentPosition});
                setRightMotorControl({POSITION, (float)profileB.currentPosition});
                
                //first get the encoder that we're comparing to get the distance. If leading encoder is 1 and we were moving forward, or if
                //leading encoder is 2 but we were moving backward, then the back encoder is b, otherwise it's A
                double encoderChosen = (leadingEncoder == 1 && forwardAligning || leadingEncoder == 2 && !forwardAligning) ? profileB.currentPosition : profileA.currentPosition;
                //now, the distance we traveled is the difference between the encoders
                double backEncoderDist = fabs(encoderChosen - backPrevDistance);
#if LOGGING_LEVEL >= 3
                serialLog("Begin encoder is: ", 2);
                serialLogln(backPrevDistance, 2);
                serialLog("End encoder is: ", 2);
                serialLogln((leadingEncoder == 1) ? profileA.currentPosition : profileB.currentPosition, 2);
#endif
                //now convert difference in ticks to meter value
                backEncoderDist *= TICK_TO_METERS;
#if LOGGING_LEVEL >= 3
                serialLog(" Encoder in front is gonna be: ", 2);
                serialLogln(leadingEncoder, 2);
                serialLog("Total distance encoder was behind is: ", 2);
                serialLogln(backEncoderDist, 2);
#endif
                //reminder: angle = arctan(x/y), where x = backEncoderDist (like distance to our edge), and y = distance between two light sensors (put in manually)

                //idk why, but when dividing by 2 that gets us the true angle. The BackEncoderDist and lightDist seems to be correct vales, but the value we
                //end up getting from it is double what it should be. Maybe there's an issue with how I did it.
                float radAngle = atan(backEncoderDist / lightDist) / 2;

                //as a reminder, corresponding degrees = (pi/180) * x radians
                float degreesAngle = 180 / M_PI * radAngle;

                serialLog("Angle is going to be: ", 2);
                serialLog(degreesAngle, 2);
                serialLogln(" degrees.", 2);
                
                //we know positive angle = turn left, negative angle means turn right.
                //encoder 1 forward means turn left, encoder 2 forward means turn right.
                //so 2 = -1, 1 = 1. If you plug those numbers in, that's what you get.

                int8_t degreesDirection = 3 - 2 * leadingEncoder;
                serialLog("Sign is going to be: ", 2);
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
            backPrevDistance = (leftEncoderChange && !forwardAligning || rightEncoderChange && forwardAligning) ? profileA.currentPosition : profileB.currentPosition;            
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