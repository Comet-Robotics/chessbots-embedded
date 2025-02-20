#ifndef CHESSBOT_ENCODER_NEW
#define CHESSBOT_ENCODER_NEW

#include <ESP32Encoder.h>
#include "robot/encoder_new.h"
#include "utils/config.h"
#include "utils/logging.h"

ESP32Encoder encoderA;
ESP32Encoder encoderB;

void setupEncodersNew() {
    log((char *)"Setting up encoders", 3);

    // Use weak pull up pins
    ESP32Encoder::useInternalWeakPullResistors = puType::up;

    encoderA.attachFullQuad(ENCODER_A_PIN1, ENCODER_A_PIN2);
    encoderB.attachFullQuad(ENCODER_B_PIN1, ENCODER_B_PIN2);
}

void resetEncoders() {
    encoderA.clearCount();
    encoderB.clearCount();
}

void encoderLoop() {
    log((char *)("Encoder count = " + String((int32_t)encoderA.getCount()) + " " + String((int32_t)encoderB.getCount())).c_str(), 4);
}

#endif