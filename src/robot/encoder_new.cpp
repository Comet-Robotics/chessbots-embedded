#ifndef CHESSBOT_ENCODER_NEW
#define CHESSBOT_ENCODER_NEW

#include "Encoder.h"
#include "robot/encoder_new.h"
#include "utils/config.h"
#include "utils/logging.h"

Encoder EncoderA(ENCODER_A_PIN1, ENCODER_A_PIN2);
Encoder EncoderB(ENCODER_B_PIN1, ENCODER_B_PIN2);
long oldPositionA = -999;
long oldPositionB = -999;


void setupEncodersNew() {
    logln((char *)"Setting up encoder", 3);
    
}

void encoderLoop() {
    long newPosition_EncA = EncoderA.read();
    long newPosition_EncB = EncoderB.read();

    if(oldPositionA != newPosition_EncA){
        oldPositionA = newPosition_EncA;
        logln(newPosition_EncA, 4);
    }

    if(oldPositionB != newPosition_EncB){
        oldPositionB = newPosition_EncB;
        logln(newPosition_EncB, 4);
    }
}

#endif