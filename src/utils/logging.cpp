#ifndef CHESSBOT_LOGGING_CPP
#define CHESSBOT_LOGGING_CPP

#include "utils/logging.h"

#include <Arduino.h>

#include "utils/status.h"
#include "../env.h"

namespace ChessBot
{
    // Prints a value or message through Serial. (The console)
    // ln means it sends a new newline character

    void log(char message[], int loggingLevel) {
        if (loggingLevel <= LOGGING_LEVEL) Serial.print(message);
    }

    void log(int value, int loggingLevel) {
        if (loggingLevel <= LOGGING_LEVEL) Serial.print(value);
    }

    void log(float value, int loggingLevel) {
        if (loggingLevel <= LOGGING_LEVEL) Serial.print(value);
    }

    void logln(char message[], int loggingLevel) {
        if (loggingLevel <= LOGGING_LEVEL) Serial.println(message);
    }

    void logln(int value, int loggingLevel) {
        if (loggingLevel <= LOGGING_LEVEL) Serial.println(value);
    }

    void logln(float value, int loggingLevel) {
        if (loggingLevel <= LOGGING_LEVEL) Serial.println(value);
    }

    void logln(std::string value, int loggingLevel) {
        if (loggingLevel <= LOGGING_LEVEL) Serial.println(value.c_str());
    }

    // This is used for specifically logging errors
    void logError(char message[], int error) {
        if (LOGGING_LEVEL > 0) Serial.printf(message, error);
    }
};

#endif