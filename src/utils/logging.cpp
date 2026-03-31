#ifndef CHESSBOT_LOGGING_CPP
#define CHESSBOT_LOGGING_CPP

#include <stdio.h>

// Associated Header File
#include "utils/logging.h"

// Built-In Libraries
#include <Arduino.h>

// Custom Libraries
#include "utils/status.h"
#include "../env.h"

// Prints a value or message through Serial. (The console)
// ln means it sends a new newline character

enum DebugLevel {
    NONE,
    INFO,
    DEBUG,
    TRACE,
    RIDICULOUS, // Use if insane
};

void serial_printf(enum DebugLevel level, const char* fmt, ...) {
    char buf[256];
    
    if level > LOGGING_LEVEL {
        return;
    }

    va_list args;
    va_start(args, fmt);

    vsnprintf(buf, sizeof(buf) fmt, args)

    Serial.print(buf)
}

void serialLog(const char *message, int serialLoggingLevel)
{
    if (serialLoggingLevel <= LOGGING_LEVEL)
        Serial.print(message);
}

void serialLog(int value, int serialLoggingLevel)
{
    if (serialLoggingLevel <= LOGGING_LEVEL)
        Serial.print(value);
}

void serialLog(double value, int serialLoggingLevel)
{
    if (serialLoggingLevel <= LOGGING_LEVEL)
        Serial.print(value);
}

void serialLog(std::string value, int serialLoggingLevel)
{
    serialLog(value.c_str(), serialLoggingLevel);
}

void serialLogln(const char *message, int serialLoggingLevel)
{
    if (serialLoggingLevel <= LOGGING_LEVEL)
        Serial.println(message);
}

void serialLogln(int value, int serialLoggingLevel)
{
    if (serialLoggingLevel <= LOGGING_LEVEL)
        Serial.println(value);
}

void serialLogln(double value, int serialLoggingLevel)
{
    if (serialLoggingLevel <= LOGGING_LEVEL)
        Serial.println(value);
}

void serialLogln(float value, int serialLoggingLevel)
{
    if (serialLoggingLevel <= LOGGING_LEVEL)
        Serial.println(value);
}

void serialLogln(std::string value, int serialLoggingLevel)
{
    serialLogln(value.c_str(), serialLoggingLevel);
}

// This is used for specifically serialLogging errors
void serialLogError(char message[], int error)
{
    if (LOGGING_LEVEL > 0)
        Serial.printf(message, error);
}

#endif