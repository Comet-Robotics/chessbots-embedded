#ifndef CHESSBOT_LOGGING_H
#define CHESSBOT_LOGGING_H

// Built-In Libraries
#include <string>

namespace ChessBot
{
    void log(char message[], int loggingLevel);
    void log(int value, int loggingLevel);

    void logln(char message[], int loggingLevel);
    void logln(int value, int loggingLevel);
    void logln(float value, int loggingLevel);
    void logln(std::string value, int loggingLevel);

    void logError(char message[], int error);
};

#endif