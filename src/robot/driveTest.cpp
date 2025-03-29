#ifndef CHESSBOT_DRIVETEST_CPP
#define CHESSBOT_DRIVETEST_CPP

#include "robot/driveTest.h"
#include "Arduino.h"
#include "robot/motor.h"
#include "robot/encoder_new.h"
#include "utils/timer.h"
#include "utils/logging.h"

void MotorEncoderTest::init()
{
    serialLogln((char *)"Starting Motor & Encoder Test", 2);

    this->testSuccessful = true;

    this->startEncoderA = readEncoderA();

    setLeftPower(1);
    setRightPower(0);
    auto fp = std::bind(&MotorEncoderTest::testLeftMotor, this);
    timerDelay(1000, fp);
}

void MotorEncoderTest::testLeftMotor()
{
    int encoderAValue = readEncoderA();
    if (encoderAValue > (this->startEncoderA + this->ENCODER_TOLERANCE))
    {
        serialLogln((char *)"Encoder A is aligned with left motor. (Positive motor = positive encoder = CW from Motor POV)", 2);
    }
    else if (encoderAValue < (this->startEncoderA - this->ENCODER_TOLERANCE))
    {
        serialLogln((char *)"[WARN] Encoder A is inverted with left motor!", 2);
        this->testSuccessful = false;
    }
    else
    {
        serialLogln((char *)"[ERR] Encoder A is not associated with left motor!", 2);
        this->testSuccessful = false;
    }
    setLeftPower(0);
    setRightPower(0);
    auto fp = std::bind(&MotorEncoderTest::testWait, this);
    timerDelay(2000, fp);
}

void MotorEncoderTest::testWait()
{
    this->startEncoderB = readEncoderB();

    setLeftPower(0);
    setRightPower(1);
    auto fp = std::bind(&MotorEncoderTest::testRightMotor, this);
    timerDelay(1000, fp);
}

void MotorEncoderTest::testRightMotor()
{
    int encoderBValue = readEncoderB();
    if (encoderBValue > (this->startEncoderB + this->ENCODER_TOLERANCE))
    {
        serialLogln((char *)"Encoder B is aligned with right motor. (Positive motor = positive encoder = CW from Motor POV)", 2);
    }
    else if (encoderBValue < (this->startEncoderB - this->ENCODER_TOLERANCE))
    {
        serialLogln((char *)"[WARN] Encoder B is inverted with right motor!", 2);
        this->testSuccessful = false;
    }
    else
    {
        serialLogln((char *)"[ERR] Encoder B is not associated with right motor!", 2);
        this->testSuccessful = false;
    }
    setRightPower(0);
    if (this->testSuccessful)
    {
        serialLogln((char *)"Test successful!", 2);
        serialLogln((char *)"To go forward, left should be set positive and right should be set negative.", 2);
        auto fp = std::bind(&MotorEncoderTest::testDriveForward, this);
        timerDelay(2000, fp);
    }
    else
    {
        serialLogln((char *)"Test failed for one or more reasons, listed above.", 2);
    }
}

void MotorEncoderTest::testDriveForward()
{
    serialLogln((char *)"Setting motors to go 'forward' for 7 seconds...", 2);
    setLeftPower(1);
    setRightPower(-1);
    auto fp = std::bind(&MotorEncoderTest::testDriveDone, this);
    timerDelay(7000, fp);
}

void MotorEncoderTest::testDriveDone()
{
    serialLogln((char *)"Driving done!", 2);
    setLeftPower(0);
    setRightPower(0);
}

#endif