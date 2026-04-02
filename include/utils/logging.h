#ifndef CHESSBOT_LOGGING_H
#define CHESSBOT_LOGGING_H

// Built-In Libraries
#include <string>

enum DebugLevel {
    NONE,
    INFO,
    DEBUG,
    TRACE,
    RIDICULOUS, // Use if insane
};

void serialLog(const char *message, int serialLoggingLevel);
void serialLog(int value, int serialLoggingLevel);
void serialLog(double value, int serialLoggingLevel);
void serialLog(std::string value, int serialLoggingLevel);

void serialLogln(const char *message, int serialLoggingLevel);
void serialLogln(int value, int serialLoggingLevel);
void serialLogln(double value, int serialLoggingLevel);
void serialLogln(float value, int serialLoggingLevel);
void serialLogln(std::string value, int serialLoggingLevel);

void serialLogError(char message[], int error);

void serial_printf(enum DebugLevel level, const char* fmt, ...);
void serial_clear();
#endif