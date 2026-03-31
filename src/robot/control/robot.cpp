#ifndef CHESSBOT_CONTROL_CPP
#define CHESSBOT_CONTROL_CPP

// Associated Header File
#include "robot/control/robot.h"

// Built-In Libraries
#include "Arduino.h"
#include <queue>

// Custom Libraries
#include "utils/logging.h"
#include "utils/timer.h"
#include "utils/config.h"
#include "utils/status.h"
#include "robot/control/motor.h"
#include "robot/control/lights.h"
#include "wifi/connection.h"
#include "robot/pidController.h"
#include "robot/driveTest.h"
#include "../../../env.h"
#include <algorithm>
#include "utils/functions.h"

#include "robot/profiledPIDController.h"
#include "robot/trapezoidalProfileNew.h"

// pid constants
TrapezoidProfile::Constraints profileConstraints(VELOCITY_LIMIT_TPS, ACCELERATION_LIMIT_TPSPS);
TrapezoidProfile xProfile(profileConstraints);
TrapezoidProfile yProfile(profileConstraints);
TrapezoidProfile aProfile(profileConstraints);
TrapezoidProfile::State xSetpoint, ySetpoint, aSetpoint;
PIDController XVelocityController(0.000009, 0.000035, 0.000000000001, -1, +1, 10); 
PIDController YVelocityController(0.0000005, 0.000035, 0.000000000001, -1, +1, 10); 
PIDController AVelocityController(0.000012, 0.00000, 0.0000000000000, -1, +1, 10); //angular velocity used for turns
ContinuousPIDController headingController(0.21, 0.000001, 0.000000000001, -1, +1, 0.1, -180, 180); // input degrees, output duty cycle

//put this in manually for each bot. Dist between the two front encoders, or the two back encoders. In meters.
const float lightDist = 0.07;
//multiply any ticks by this ratio to get distance in meters
const float TICK_TO_METERS = TIRE_CIRCUMFERENCE / TICKS_PER_ROTATION;

// const uint8_t Top_Left_Encoder_Index = 0;
// const uint8_t Top_Right_Encoder_Index = 1;
// const uint8_t Bottom_Left_Encoder_Index = 2;
// const uint8_t Bottom_Right_Encoder_Index = 3;

// boolean testEncoderPID_value = false;

// //determines the encoder values the iteration right before
// int prevPositionL = 0;
// int prevPositionR = 0;
// int timeMs = 0;

// //robot rotation in radians
// double prevRotation = 0;

// //calculated previous X and Y values
// double prevX = 0;
// double prevY = 0;

// bool runningPID = false;
// bool runningTurn = false;
// double average[50];

// //when moving to a different target, this is the encoder position we started from
// int startEncoderAPos = -1;
// int startEncoderBPos = -1;

// //the encoder distance to about halfway through the tile
// int encoderAHalfwayDist = 0;
// int encoderBHalfwayDist = 0;

// //when we receive a command from the website to center, set this from false to true,
// //then when the code sets it back to false, that's when we know we're done centering
// bool isCentering = false;

// //holds the current status representing the progress in centering. The different values
// //are defined below
// char centeringStatus = 'S';

// //measures if the forward encoders are aligning on the edge or the back encoders
// bool forwardAligning = true;

// //set this true when our robot is on an edge and is moving to the center from its given measured distance
// bool movingCenter = false;

// //set this true when our robot is turning to align itself on the next axis
// bool turningToNextAxis = false;

// //initially 0, once 2 means both axises aligned so we done
// uint8_t axisesAligned = 0;

// float angle = 0;
// unsigned long timeSinceTurn = 0;

// //the value of the encoder we're measuring in terms of its light value
// bool firstEncoderVal = false;
// bool secondEncoderVal = false;

// //represents which index is being referenced for the current encoders
// uint8_t firstEncoderIndex = 0;
// uint8_t secondEncoderIndex = 0;

// //Here are the possible values:
//     //0 = no encoder leading.
//     //1 = left encoder leading.
//     //2 = right encoder leading
// uint8_t leadingEncoder = 0;

// //previous encoder distance, specifically the one that is lagging behind
// float backPrevDistance = 0;

// //checks if the given encoders have changed tiles yet.
// bool leftEncoderChange = false;
// bool rightEncoderChange = false;

// //determines basically what tile each encoder is one. false is one tile, true is the opposite tile.
// //it could be false = white and true = black, or false = black and white = true, doesn't really matter
// bool onFirstTile[4] = {false, false, false, false};

// bool waitingForLight = false;

Robot::Robot() {

        }

int Robot::batteryLevel() {
            return analogRead(BATTERY_VOLTAGE_PIN) - BATTERY_VOLTAGE_OFFSET;
        }

void Robot::tick() {
    center_tick();
    pid_tick();
    turn_tick();
}

void Robot::pid_tick() {
            double lDist = (double)(readLeftEncoder()-prevPositionL)/TICKS_PER_ROTATION*2*PI;
            double rDist = (double)(readRightEncoder()-prevPositionR)/TICKS_PER_ROTATION*2*PI;

            prevPositionL = readLeftEncoder();
            prevPositionR = readRightEncoder();

            double angleDist = (rDist - lDist)*TIRE_RADIUS/trackWidth;

            double currentRot = prevRotation + angleDist;

            double currentX;
            double deltaX;
            double currentY;
            double deltaY;
            if(angleDist != 0){
                double temp = (trackWidth*(rDist+lDist))/(2*(rDist-lDist));
                deltaX = temp * (sin(currentRot) - sin(prevRotation));
                deltaY = temp * (cos(prevRotation) - cos(currentRot));
                currentX = prevX + deltaX;
                currentY = prevY + deltaY;
                
            } else {
                double totalDist = TIRE_RADIUS*(lDist + rDist)/2;
                deltaX = totalDist * cos(currentRot);
                deltaY = totalDist * sin(currentRot);
                currentX = prevX + deltaX;
                currentY = prevY + deltaY;
            }
            prevX = currentX;
            prevY = currentY;
            prevRotation = currentRot;

            double loopDelaySeconds = ((double) timeMs) / 1000;
            profileX.currentPosition = currentX / TICK_TO_METERS;
            profileX.currentVelocity = deltaX==0?0:deltaX / TICK_TO_METERS /loopDelaySeconds;
            xSetpoint = xProfile.calculate(loopDelaySeconds, 
                                                    TrapezoidProfile::State(profileX.currentPosition, profileX.currentVelocity),
                                                    TrapezoidProfile::State(getLeftMotorControl().value, 0.0));

            double velocity = XVelocityController.Compute(xSetpoint.velocity, profileX.currentVelocity, loopDelaySeconds);
            double aVel = headingController.Compute(0, currentRot*RAD_TO_DEG, loopDelaySeconds);

            profileY.currentPosition = currentY / TICK_TO_METERS;
            profileY.currentVelocity = deltaY==0?0:deltaY / TICK_TO_METERS /loopDelaySeconds;
            ySetpoint = yProfile.calculate(loopDelayMs/1000, 
                                                    TrapezoidProfile::State(profileY.currentPosition, profileY.currentVelocity),
                                                    TrapezoidProfile::State(getRightMotorControl().value, 0.0));
            double yVel = YVelocityController.Compute(ySetpoint.velocity, profileY.currentVelocity, loopDelaySeconds); 

            aVel += yVel;

            double lMotorPower = fmap((velocity-8*aVel*trackWidth/2)/TIRE_RADIUS, -25, 25, -1, 1);
            double rMotorPower = fmap((velocity+8*aVel*trackWidth/2)/TIRE_RADIUS, -25, 25, -1, 1);

            if (fabs(lMotorPower) < MIN_MOTOR_POWER) lMotorPower = 0;
            if (fabs(rMotorPower) < MIN_MOTOR_POWER) rMotorPower = 0;


            serial_printf(
                DebugLevel::DEBUG,
                "vel %f yvel %f avel %f lpower %f rpower %f xpos %f ypos %f\n",
                velocity,
                yVel,
                aVel,
                lMotorPower,
                rMotorPower,
                currentX,
                currentY
            );

            setLeftPower(lMotorPower);
            setRightPower(rMotorPower);

            double sum = 0;
            for(int x = 49; x >= 0; x--) {
                average[x+1] = average[x];
                sum += fabs(average[x+1]);
            }
            average[0] = lMotorPower;
            sum += fabs(average[0]);
            timeMs += loopDelayMs;
            if (sum/50 < 0.01){
                serialLogln(sum/50,3);
                serialLogln(runningPID,3);
                runningPID = false;
                setLeftPower(0);
                setRightPower(0);

                sendActionSuccess("move done");
                resetPID();

            } else {
                runningPID = true;
            }
}

void Robot::turn_tick() {
    double lDist = left.dist();
    double rDist = right.dist();

    prevPositionL = readLeftEncoder();
    prevPositionR = readRightEncoder();

    double angleDist = (rDist - lDist)*TIRE_RADIUS/trackWidth;

    double currentRot = prevRotation + angleDist;

    double currentX;
    double deltaX;
    double currentY;
    double deltaY;
    if(angleDist != 0){
        double temp = (trackWidth*(rDist+lDist))/(2*(rDist-lDist));
        deltaX = temp * (sin(currentRot) - sin(prevRotation));
        deltaY = temp * (cos(prevRotation) - cos(currentRot));
        currentX = prevX + deltaX;
        currentY = prevY + deltaY;
        
    } else {
        double totalDist = TIRE_RADIUS*(lDist + rDist)/2;
        deltaX = totalDist * cos(currentRot);
        deltaY = totalDist * sin(currentRot);
        currentX = prevX + deltaX;
        currentY = prevY + deltaY;
    }
    prevX = currentX;
    prevY = currentY;

    prevRotation = currentRot;

    double loopDelaySeconds = ((double) timeMs) / 1000;
    profileA.currentPosition = currentRot*10 / TICK_TO_METERS;
    profileA.currentVelocity = angleDist==0?0:angleDist*10 / TICK_TO_METERS /loopDelaySeconds;
    aSetpoint = aProfile.calculate(loopDelaySeconds, 
                                            TrapezoidProfile::State(profileA.currentPosition, profileA.currentVelocity),
                                            TrapezoidProfile::State(getHeadingTarget()*10/TICK_TO_METERS, 0.0));

    double velocity = AVelocityController.Compute(aSetpoint.velocity, profileA.currentVelocity, loopDelaySeconds);

    double lMotorPower = fmap((-8*velocity*trackWidth/2)/TIRE_RADIUS, -11, 11, -1, 1);
    double rMotorPower = fmap((8*velocity*trackWidth/2)/TIRE_RADIUS, -11, 11, -1, 1);

    if (fabs(lMotorPower) < MIN_MOTOR_POWER) lMotorPower = 0;
    if (fabs(rMotorPower) < MIN_MOTOR_POWER) rMotorPower = 0;

    setLeftPower(lMotorPower);
    setRightPower(rMotorPower);

    serialLog("vels ", 3);
    serialLog(xSetpoint.velocity, 3);
    serialLog("vel ", 3);
    serialLog(velocity, 3);
    serialLog(" lpower ", 3);
    serialLog(lMotorPower, 3);
    serialLog(" rpower ", 3);
    serialLog(rMotorPower, 3);
    serialLog(" angle ", 3);
    serialLogln(currentRot*10, 3);


    double sum = 0;
    for(int x = 49; x >= 0; x--) {
        average[x+1] = average[x];
        sum += fabs(average[x+1]);
    }
    average[0] = lMotorPower;
    sum += fabs(average[0]);
    timeMs += loopDelayMs;
    if (sum/50 < 0.01){
        serialLogln(sum/50,3);
        serialLogln(runningPID,3);
        runningPID = false;
        setLeftPower(0);
        setRightPower(0);

        sendActionSuccess("turn done");
        resetPID();

    } else {
        runningTurn = true;
    }
}

void Robot::center() {
    centeringStatus = STARTED;
}

void Robot::drive(float tiles, std::string id) {
    if (!getStoppedStatus()) {
        const float TILE_SIZE_INCHES = 24;
        float distanceInches = tiles * TILE_SIZE_INCHES;
        float ticksPerInch = TICKS_PER_ROTATION / (WHEEL_DIAMETER_INCHES * M_PI);
        int tickDistance = (int)(distanceInches * ticksPerInch);
        driveTicks(tickDistance, id);
    }
}

// Drives the wheels according to the powers set. Negative is backwards, Positive forwards
void Robot::drive(float leftPower, float rightPower, std::string id) {
    if (!getStoppedStatus()) {
        if (leftPower < MIN_MOTOR_POWER && leftPower > -MIN_MOTOR_POWER) {
            leftPower = 0;
        } if (rightPower < MIN_MOTOR_POWER && rightPower > -MIN_MOTOR_POWER) {
            rightPower = 0;
        }

        setLeftPower(leftPower);
        setRightPower(rightPower);

        //we only send null as id during our test drive. The only other time this drive method is called will be
        //when the server sends it, meaning it will have an id to send back.
        if (id != "NULL") { sendActionSuccess(id); }
    }
        }

void Robot::driveTicks(int tickDistance, std::string id) {
    if (stopped) {
        resetSpeed();
        setXControl({POSITION, (float)tickDistance});
        setYControl({POSITION, 0});
        updateCritRange();
        runningPID = true;

        if (id != "NULL") {
            sendPacketOnPidComplete(id);
        }
    }
    }

//turns the given amount in radians, CCW
void Robot::turn(float angleRadians, std::string id) {
    serialLogln("Turning", 3);
    serialLogln(angleRadians, 3);

    setXControl({POSITION, 0});
    setYControl({POSITION, 0});
    updateCritRange();
    setHeadingTarget(angleRadians*0.95);
    runningTurn = true;

    if (id != "NULL")
    {
        sendPacketOnPidComplete(id);
    }
}

void Robot::stop() {
    stopped = true;

    left.power(0);
    right.power(0);

    serialLogln("Bot Stopped!", 2);
}



/**
 * get PID ready for the next iteration
 */
void resetPID(){
    prevPositionL = readLeftEncoder();
    prevPositionR = readRightEncoder();
    prevX = 0;
    prevY = 0;
    prevRotation = 0;
    timeMs = 0;
    for (int x = 0; x < 50; x++) average[x] = 100;
}


//crit range is basically getting distance we go until we are halfway to target. By the end of this range,
//we're at our max speed and are sure we aren't at a low speed just cause we're speeding up
void updateCritRange()
{
    //want to record the values before we move now
    startEncoderAPos = profileX.currentPosition;
    startEncoderBPos = profileY.currentPosition;
    
    profileX.criticalRange = fabs(profileX.targetPosition - startEncoderAPos) / 2;
    profileY.criticalRange = fabs(profileY.targetPosition - startEncoderBPos) / 2;

    //update it so that time since start is now equal to this. Only really care about this value after turns though
    timeSinceTurn = millis();
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

    XVelocityController.Reset();
    YVelocityController.Reset();
    AVelocityController.Reset();
    headingController.Reset();
    resetPID();


    // if (DO_PID_TEST) {
    //     testEncoderPID();
    //     timerInterval(8000, &testEncoderPID);
    // }

    // if (DO_TURN_TEST) {
    //     angle = 30;
    //     testTurn();
    //     timerInterval(5000, testTurn);
    // }

    // if (DO_CENTERING_TEST) {
    //     testCentering();
    //     timerInterval(5000, &testCentering);
    // }
}

//this determines what our next action will be after an edge alignment, move to center, or axis turn.
void nextAction()
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
            centeringStatus = CenteringStatus::NOT_CENTERING;
        }
        //otherwise restart the whole process for this next axis
        else
        {
            centeringStatus = CenteringStatus::STARTED;
        }
    }
    //if previously moving to center for an axis, and finished:
    else if(movingCenter)
    {
        //now, turn to next direction
        movingCenter = false;
        turningToNextAxis = true;
        centeringStatus = CenteringStatus::MOVING;
        //this way, if no axises aligned yet, angle = -90, and if one axis aligned,
        //angle = 90, undoing the previous rotation.
        angle = 90 * (axisesAligned * 2 - 1);

        // testTurn();
    }
    //if just aligned on a forward edge with the encoders in the front:
    else if(forwardAligning)
    {
        //if going forward, store the current encoder value so we can see the full encoder length of the tiles
        encoderAHalfwayDist = profileX.currentPosition;
        encoderBHalfwayDist = profileY.currentPosition;
        //swap to going back
        forwardAligning = !forwardAligning;
        //set that new drive
        createDriveUntilNewTile();
        centeringStatus = 'E';

        serialLog((char*)"Current encoder A: ", 2);
        serialLogln(profileX.currentPosition, 2);
        serialLog((char*)"Current encoder B: ", 2);
        serialLogln(profileY.currentPosition, 2);
        serialLog((char*)"Target encoder A: ", 2);
        // serialLogln(leftMotorControl.mode == POSITION ? leftMotorControl.value : 0, 2);
        serialLog((char*)"Target encoder B: ", 2);
        // serialLogln(rightMotorControl.mode == POSITION ? rightMotorControl.value : 0, 2);
        }
    //if just aligned on a backward edge with the encoders in the back:
    else
    {
        //swap to going forward now
        forwardAligning = !forwardAligning;
        
        //since we always go forward first and then backwards, the current value of encoderHalfwayDist > currentEncoder always
        //what we're doing is currently, "encoderAHalfwayDist" just stores the value of the other edge in encoder ticks, now we're
        //finding the difference between them. And of course divide by 2 as want half that distance
        encoderAHalfwayDist = (encoderAHalfwayDist - profileX.currentPosition) / 2;
        encoderBHalfwayDist = (encoderBHalfwayDist - profileY.currentPosition) / 2;

        //decide that we take average of encoder A and B's distances, since ideally we want both to travel the same amount
        int totalHalfwayDistance = (encoderAHalfwayDist + encoderBHalfwayDist) / 2;
        // driveTicks(totalHalfwayDistance, "NULL");
        centeringStatus = CenteringStatus::MOVING;
        //now we moving to da center
        movingCenter = true;
    }
}

//this updates our centering depending on the current status of it
void Robot::center_tick()
{
    //what we do depend on the current status
    switch(centeringStatus)
    {
        case CenteringStatus::STARTED:
            //create the first drive
            createDriveUntilNewTile();
            //now change it so we're driving forward
            centeringStatus = CenteringStatus::EDGE;
            break;
        case CenteringStatus::EDGE:
        {
            //continue driving forward
            DriveStatus drive_status = driveUntilNewTile();
            //reminder status 3 = no leading encoders and driving finished
            if(driveStatus == DriveStatus::REACHED)
            {
                //meaning we can now go in opposite direction.
                nextAction();
            }
            //reminder status 2 = one leading encoder finished first, but driving is finished
            else if(driveStatus == == DriveStatus::REACHED_REVERSING)
            {
                //mean we now begin correcting
                centeringStatus = CenteringStatus::MOVING; // ????
            }
            break;
        }
        case CenteringStatus::MOVING:
            //check if we're the moving we're doing is finished. If so, determine what we do next.
            
            // ???
            // if(checkMoveFinished())
            // {
            //     nextAction();
            // }
            break;
    }
}

//checks if we're done moving to our target when either moving half a tile's length or turning
// bool checkMoveFinished()
// {
//     //first, checks like "fabs(profileX.currentVelocity) < 2" see if we're slowing down or not.

//     //then, something like "fabs(currentEncoderA - encoderATarget) < criticalRangeA" is making sure
//     //the reason our speed is slow is specifically because we're slowing down and not because we're
//     //beginning to speed up.
//     //do this by seeing if the distance remaining is less than the midpoint distance from start to end, as by then we're at our max speed.

//     bool encoderAChecks = fabs(profileX.currentPosition - profileX.targetPosition) < profileX.criticalRange && fabs(profileX.currentVelocity) < 3;
//     bool encoderBChecks = fabs(profileY.currentPosition - profileY.targetPosition) < profileY.criticalRange  && fabs(profileY.currentVelocity) < 3;

//     //check if we've been stalling too long, for 8 seconds. If we're over time, that's bad, and means we should declare the movement finished.
//     bool timerCheck = millis() - timeSinceTurn > 8000;
    
//     return ((encoderAChecks && encoderBChecks) || timerCheck);
// }

// //like the one above but just seeing if we can keep moving or not
// bool checkIfCanUpdateMovement()
// {
//     return fabs(profileX.currentPosition - profileX.targetPosition) < profileX.criticalRange && fabs(profileY.currentPosition - profileY.targetPosition) < profileY.criticalRange;
// }

boolean isRobotPidAtTarget() {
    if (!DO_PID)
        return true;

    boolean leftAtTarget, rightAtTarget;

    if (getLeftMotorControl().mode == POSITION)
    {
        leftAtTarget = approxEquals(getLeftMotorControl().value, profileX.currentPosition, PID_POSITION_TOLERANCE)
                    && approxEquals(profileX.currentVelocity, 0.0, PID_VELOCITY_TOLERANCE);

        serialLog("Left Position: ", 3);
        serialLog(profileX.currentPosition, 3);
        serialLog(", Target: ", 3);
        serialLog(getLeftMotorControl().value, 3);
        serialLog(", Velocity: ", 3);
        serialLog(profileX.currentVelocity, 3);
        serialLog(", At Target: ", 3);
        serialLogln(leftAtTarget, 3);
    }
    else
    {
        leftAtTarget = approxEquals(getLeftMotorControl().value, profileX.currentVelocity, PID_VELOCITY_TOLERANCE);
    }
    if (getRightMotorControl().mode == POSITION)
    {
        rightAtTarget = approxEquals(getRightMotorControl().value, profileY.currentPosition, PID_POSITION_TOLERANCE)
                     && approxEquals(profileY.currentVelocity, 0.0, PID_VELOCITY_TOLERANCE);

        serialLog("Right Position: ", 3);
        serialLog(profileY.currentPosition, 3);
        serialLog(", Target: ", 3);
        serialLog(getRightMotorControl().value, 3);
        serialLog(", Velocity: ", 3);
        serialLog(profileY.currentVelocity, 3);
        serialLog(", At Target: ", 3);
        serialLogln(rightAtTarget, 3);
    }
    else
    {
        rightAtTarget = approxEquals(getRightMotorControl().value, profileY.currentVelocity, PID_VELOCITY_TOLERANCE);
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

// //updates us to the next distance we're traveling
// void updateToNextDistance()
// {
//     //this way if we're reversing, we're actually subtracting. if going forward was 0, then 0 * 2 - 1 = -1.
//     //if going forward was 1, 1 * 2 - 1 = 1.
//     float encTargetA = (float)profileX.currentPosition + 2500 * (forwardAligning * 2 - 1);
//     float encTargetB = (float)profileY.currentPosition + 2500 * (forwardAligning * 2 - 1);
//     setXControl({POSITION, encTargetA});
//     setYControl({POSITION, encTargetB});
// #if LOGGING_LEVEL >= 3
//     serialLogln("changing direction!", 3);
//     serialLogln(encTargetA, 3);
//     serialLogln(encTargetB, 3);
// #endif

//     //update crit range yessir
//     updateCritRange();
// }

//creates a new drive until the next tile
// void createDriveUntilNewTile()
// {
//     //set the next distance to travel
//     resetSpeed();
//     updateToNextDistance();
    
//     //assign values here, will detect when they change
//     if(forwardAligning)
//     {
//         firstEncoderIndex = Top_Left_Encoder_Index;
//         secondEncoderIndex = Top_Right_Encoder_Index;
//     }
//     else
//     {
//         //from the robots perspective as it's going backwards, the encoder on the left is bottom right, 
//         //while the encoder on the right is bottom left, so we just swap em
//         firstEncoderIndex = Bottom_Right_Encoder_Index;
//         secondEncoderIndex = Bottom_Left_Encoder_Index;
//     }
//     //now set the actual values with the indices
//     firstEncoderVal = onFirstTile[firstEncoderIndex];
//     secondEncoderVal = onFirstTile[secondEncoderIndex];
// }

//drives until the first two encoders are considered to hit a new value.
//Returns a status bit, with the corresponding values:
    // 1 - drive is going
    // 2 - just at this moment, we have reached our destination and are going to begin reversing now.
    // 3 - just at this moment, we've reached our destination, but DON'T need to reverse.
enum DriveStatus Robot::driveUntilNewTile() 
{
    //if we do finish moving, update to a new distance. I think we have to move at intervals for it to work
    if(checkIfCanUpdateMovement())
    {
        updateToNextDistance();
    }
    
    //if we already changed it, don't change it back again
    leftEncoderChange = leftEncoderChange || (onFirstTile[firstEncoderIndex] != firstEncoderVal);
    rightEncoderChange = rightEncoderChange || (onFirstTile[secondEncoderIndex] != secondEncoderVal);

    if(!leftEncoderChange && !rightEncoderChange) {
        return CenteringStatus::STARTED;
    }

    //when both cross, we done.
    if(leftEncoderChange && rightEncoderChange)
    {
        //if there had been a leading encoder, do the following:
        uint8_t status = 0; 
        if(leadingEncoder != 0)
        {
            setXControl({POSITION, (float)profileX.currentPosition});
            setYControl({POSITION, (float)profileY.currentPosition});
            
            //first get the encoder that we're comparing to get the distance. If leading encoder is 1 and we were moving forward, or if
            //leading encoder is 2 but we were moving backward, then the back encoder is b, otherwise it's A
            double encoderChosen = (leadingEncoder == 1 && forwardAligning || leadingEncoder == 2 && !forwardAligning) ? profileY.currentPosition : profileX.currentPosition;
            //now, the distance we traveled is the difference between the encoders
            double backEncoderDist = fabs(encoderChosen - backPrevDistance);
            // serialLog("Begin encoder is: ", 3);
            // serialLogln(backPrevDistance, 3);
            // serialLog("End encoder is: ", 3);
            // serialLogln((leadingEncoder == 1) ? profileX.currentPosition : profileY.currentPosition, 3);
            //now convert difference in ticks to meter value
            backEncoderDist *= TICK_TO_METERS;
            // serialLog(" Encoder in front is gonna be: ", 3);
            // serialLogln(leadingEncoder, 2);
            // serialLog("Total distance encoder was behind is: ", 2);
            // serialLogln(backEncoderDist, 2);
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
            // testTurn();
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
        backPrevDistance = (leftEncoderChange && !forwardAligning || rightEncoderChange && forwardAligning) ? profileX.currentPosition : profileY.currentPosition;            
    }
    
    return 1;
}

// // Tests the motors. This turns the motors off.
// void driveTestOff() {
//     stop();
//     timerDelay(2000, &startDriveTest);
// }

// // Test motor and encoder synchronization
// void startMotorAndEncoderTest() {
//     (new MotorEncoderTest())->startMotorAndEncoderTest();
// }

// // Tests the motors. This turns the motors on.
// void startDriveTest() {
//     drive(0.5f, 0.5f, "NULL");
//     //timerDelay(2000, &driveTestOff);
// }

// void testCentering()
// {
//     serialLogln("Running centering routine...", 2);
//     startCentering();
// }

// void testEncoderPID()
// {
//     serialLogln("Changing encoder PID setpoint!", 2);
//     if (!testEncoderPID_value)
//     {
//         testEncoderPID_value = true;
//         setXControl({POSITION, (float)TICKS_PER_ROTATION * 6});
//         setYControl({POSITION, 0.0f});
//         runningPID = true;
//     }
//     else
//     {
//         testEncoderPID_value = false;
//         setXControl({POSITION, 0.0f});
//         setYControl({POSITION, 0.0f});
//         runningPID = true;
//     }

//     updateCritRange();
// }

// void testTurn()
// {
//     resetSpeed();
    
//     serialLog("Changing destination angle to ", 2);
//     serialLog(angle, 2);
//     //simple maths
//     turn(M_PI / 180 * angle, "NULL");
//     serialLog(" (", 2);
//     serialLog(getLeftMotorControl().value, 2);
//     serialLog(", ", 2);
//     serialLog(getRightMotorControl().value, 2);
//     serialLogln(")", 2);

//     //compute the new crit range now that target and start encoder have changed
//     updateCritRange();
// }
#endif