#ifndef CHESSBOT_LOGGING_CPP
#define CHESSBOT_LOGGING_CPP

// Associated Header File
#include "utils/logging.h"

// Built-In Libraries
#include <Arduino.h>

// Custom Libraries
#include "utils/status.h"
#include "../env.h"

// Prints a value or message through Serial. (The console)
// ln means it sends a new newline character

void serialLog(char message[], int serialLoggingLevel) {
    if (serialLoggingLevel <= LOGGING_LEVEL) Serial.print(message);
}

void serialLog(int value, int serialLoggingLevel) {
    if (serialLoggingLevel <= LOGGING_LEVEL) Serial.print(value);
}

void serialLog(float value, int serialLoggingLevel) {
    if (serialLoggingLevel <= LOGGING_LEVEL) Serial.print(value);
}

void serialLogln(char message[], int serialLoggingLevel) {
    if (serialLoggingLevel <= LOGGING_LEVEL) Serial.println(message);
}

void serialLogln(int value, int serialLoggingLevel) {
    if (serialLoggingLevel <= LOGGING_LEVEL) Serial.println(value);
}

void serialLogln(float value, int serialLoggingLevel) {
    if (serialLoggingLevel <= LOGGING_LEVEL) Serial.println(value);
}

void serialLogln(std::string value, int serialLoggingLevel) {
    if (serialLoggingLevel <= LOGGING_LEVEL) Serial.println(value.c_str());
}

// This is used for specifically serialLogging errors
void serialLogError(char message[], int error) {
    if (LOGGING_LEVEL > 0) Serial.printf(message, error);
}

#endif