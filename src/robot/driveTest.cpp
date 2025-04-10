#ifndef CHESSBOT_DRIVETEST_CPP
#define CHESSBOT_DRIVETEST_CPP

#include "robot/driveTest.h"
#include "Arduino.h"
#include "robot/motor.h"
#include "robot/encoder.h"
#include "utils/timer.h"
#include "utils/logging.h"

void MotorEncoderTest::init()
{
    serialLogln("Starting Motor & Encoder Test", 2);

    this->testSuccessful = true;

    this->startEncoderA = readLeftEncoder();
    this->_startEncoderB = readRightEncoder();

    setLeftPower(1);
    setRightPower(0);
    auto fp = std::bind(&MotorEncoderTest::testLeftMotor, this);
    timerDelay(1000, fp);
}

void MotorEncoderTest::testLeftMotor()
{
    int encoderAValue = readLeftEncoder();
    if (encoderAValue > (this->startEncoderA + this->ENCODER_TOLERANCE))
    {
        serialLogln("Encoder A (lbl RightEncoder) is aligned with left motor (lbl M1). (Positive motor = positive encoder = CW from Motor POV)", 2);
    }
    else if (encoderAValue < (this->startEncoderA - this->ENCODER_TOLERANCE))
    {
        serialLogln("[WARN] Encoder A (lbl RightEncoder) is inverted with left motor (lbl M1)!", 2);
        this->testSuccessful = false;
    }
    else
    {
        serialLogln("[ERR] Encoder A (lbl RightEncoder) is not associated with left motor (lbl M1)!", 2);
        this->testSuccessful = false;
    }
    setLeftPower(0);
    setRightPower(0);
    auto fp = std::bind(&MotorEncoderTest::testWait, this);
    timerDelay(2000, fp);
}

void MotorEncoderTest::testWait()
{
    this->startEncoderB = readRightEncoder();

    setLeftPower(0);
    setRightPower(1);
    auto fp = std::bind(&MotorEncoderTest::testRightMotor, this);
    timerDelay(1000, fp);
}

void MotorEncoderTest::testRightMotor()
{
    int encoderBValue = readRightEncoder();
    if (encoderBValue > (this->startEncoderB + this->ENCODER_TOLERANCE))
    {
        serialLogln("Encoder B (lbl LeftEncoder) is aligned with right motor (lbl M2). (Positive motor = positive encoder = CCW from Motor POV)", 2);
    }
    else if (encoderBValue < (this->startEncoderB - this->ENCODER_TOLERANCE))
    {
        serialLogln("[WARN] Encoder B (lbl LeftEncoder) is inverted with right motor (lbl M2)!", 2);
        this->testSuccessful = false;
    }
    else
    {
        serialLogln("[ERR] Encoder B (lbl LeftEncoder) is not associated with right motor (lbl M2)!", 2);
        this->testSuccessful = false;
    }
    setRightPower(0);
    if (this->testSuccessful)
    {
        serialLogln("Test successful!", 2);
        serialLogln("To go forward, both left and right should be set positive.", 2);
        auto fp = std::bind(&MotorEncoderTest::testDriveForward, this);
        timerDelay(2000, fp);
    }
    else
    {
        int encoderAValue = readLeftEncoder();
        if ((this->startEncoderA - this->ENCODER_TOLERANCE) <= encoderAValue && encoderAValue <= (this->startEncoderA + this->ENCODER_TOLERANCE)) {
            serialLogln("[WARN] Encoder A (lbl RightEncoder) did not significantly change over the course of the test!", 2);
        }
        if ((this->_startEncoderB - this->ENCODER_TOLERANCE) <= encoderBValue && encoderBValue <= (this->_startEncoderB + this->ENCODER_TOLERANCE))
        {
            serialLogln("[WARN] Encoder B (lbl LeftEncoder) did not significantly change over the course of the test!", 2);
        }
        serialLogln("Test failed for one or more reasons, listed above.", 2);
        if (((this->startEncoderA - this->ENCODER_TOLERANCE) > encoderAValue || encoderAValue > (this->startEncoderA + this->ENCODER_TOLERANCE))
         && ((this->_startEncoderB - this->ENCODER_TOLERANCE) > encoderBValue || encoderBValue > (this->_startEncoderB + this->ENCODER_TOLERANCE))) {
            serialLogln("Encoder A and B changed but did not have the correct associations. Maybe swap the motors or the encoders?", 2);
        }
    }
}

void MotorEncoderTest::testDriveForward()
{
    serialLogln("Setting motors to go 'forward' for 7 seconds...", 2);
    setLeftPower(1);
    setRightPower(1);
    auto fp = std::bind(&MotorEncoderTest::testDriveDone, this);
    timerDelay(7000, fp);
}

void MotorEncoderTest::testDriveDone()
{
    serialLogln("Driving done!", 2);
    setLeftPower(0);
    setRightPower(0);
}

#endif