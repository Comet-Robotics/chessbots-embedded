#ifndef CHESSBOT_LOGGING_CPP
#define CHESSBOT_LOGGING_CPP

#include "utils/logging.h"

#include <Arduino.h>
#include <iostream>

#include "utils/status.h"

namespace ChessBot
{
    void log(char message[]) {
        if (getLoggingStatus()) {
            Serial.print(message);
        }
    }

    void log(int value) {
        if (getLoggingStatus()) {
            Serial.print(value);
        }
    }

    void log(float value) {
        if (getLoggingStatus()) {
            Serial.print(value);
        }
    }

    void logln(char message[]) {
        if (getLoggingStatus()) {
            Serial.println(message);
        }
    }

    void logln(int value) {
        if (getLoggingStatus()) {
            Serial.println(value);
        }
    }

    void logln(float value) {
        if (getLoggingStatus()) {
            Serial.println(value);
        }
    }

    void logln(std::string value) {
        if (getLoggingStatus()) {
            Serial.println(value.c_str());
        }
    }

    void logError(char message[], int error) {
        if (getLoggingStatus()) {
            Serial.printf(message, error);
        }
    }
};

#endif