#ifndef CHESSBOT_ENCODER_H
#define CHESSBOT_ENCODER_H

void setupEncodersNew();
void resetEncoders();
void encoderLoop();

int readEncoderA();
int readEncoderB();

#endif