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
    this->_startEncoderB = readEncoderB();

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
        serialLogln((char *)"Encoder A (RightEncoder) is aligned with left motor (M1). (Positive motor = positive encoder = CW from Motor POV)", 2);
    }
    else if (encoderAValue < (this->startEncoderA - this->ENCODER_TOLERANCE))
    {
        serialLogln((char *)"[WARN] Encoder A (RightEncoder) is inverted with left motor (M1)!", 2);
        this->testSuccessful = false;
    }
    else
    {
        serialLogln((char *)"[ERR] Encoder A (RightEncoder) is not associated with left motor (M1)!", 2);
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
        serialLogln((char *)"Encoder B (LeftEncoder) is aligned with right motor (M2). (Positive motor = positive encoder = CW from Motor POV)", 2);
    }
    else if (encoderBValue < (this->startEncoderB - this->ENCODER_TOLERANCE))
    {
        serialLogln((char *)"[WARN] Encoder B (LeftEncoder) is inverted with right motor (M2)!", 2);
        this->testSuccessful = false;
    }
    else
    {
        serialLogln((char *)"[ERR] Encoder B (LeftEncoder) is not associated with right motor (M2)!", 2);
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
        int encoderAValue = readEncoderA();
        if ((this->startEncoderA - this->ENCODER_TOLERANCE) <= encoderAValue && encoderAValue <= (this->startEncoderA + this->ENCODER_TOLERANCE)) {
            serialLogln((char *)"[WARN] Encoder A (RightEncoder) did not significantly change over the course of the test!", 2);
        }
        if ((this->_startEncoderB - this->ENCODER_TOLERANCE) <= encoderBValue && encoderBValue <= (this->_startEncoderB + this->ENCODER_TOLERANCE))
        {
            serialLogln((char *)"[WARN] Encoder B (LeftEncoder) did not significantly change over the course of the test!", 2);
        }
        serialLogln((char *)"Test failed for one or more reasons, listed above.", 2);
        if (((this->startEncoderA - this->ENCODER_TOLERANCE) > encoderAValue || encoderAValue > (this->startEncoderA + this->ENCODER_TOLERANCE))
         && ((this->_startEncoderB - this->ENCODER_TOLERANCE) > encoderBValue || encoderBValue > (this->_startEncoderB + this->ENCODER_TOLERANCE))) {
            serialLogln((char *)"Encoder A and B changed but did not have the correct associations. Maybe swap the motors or the encoders?", 2);
        }
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