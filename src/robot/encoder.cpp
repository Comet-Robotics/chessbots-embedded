#ifndef CHESSBOT_ENCODER_CPP
#define CHESSBOT_ENCODER_CPP

#include "Encoder.h"
#include "robot/encoder.h"
#include "utils/config.h"
#include "utils/logging.h"

Encoder EncoderA(ENCODER_A_PIN1, ENCODER_A_PIN2); // right
Encoder EncoderB(ENCODER_B_PIN1, ENCODER_B_PIN2); // left
int oldPositionA = -999;
int oldPositionB = -999;


void setupEncodersNew() {
    serialLogln((char *)"Setting up encoder", 3);
}

void resetEncoders() {
    EncoderA.write(0);
    EncoderB.write(0);
}

void encoderLoop() {
    int newPosition_EncA = readLeftEncoder();
    int newPosition_EncB = readRightEncoder();

    if(oldPositionA != newPosition_EncA){
        oldPositionA = newPosition_EncA;
        serialLog((char *)"Encoder A: ", 2);
        serialLogln(newPosition_EncA, 2);
    }

    if(oldPositionB != newPosition_EncB){
        oldPositionB = newPosition_EncB;
        serialLog((char *)"Encoder B: ", 2);
        serialLogln(newPosition_EncB, 2);
    }
}

int readLeftEncoder() {
    // TODO swap encoders and rename debug logs
    return EncoderA.read();
}

int readRightEncoder() {
    return -EncoderB.read();
}

#endif