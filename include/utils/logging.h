#ifndef CHESSBOT_LOGGING_H
#define CHESSBOT_LOGGING_H

#include <string>

namespace ChessBot
{
    void log(char message[]);
    void log(int value);

    void logln(char message[]);
    void logln(int value);
    void logln(float value);
    void logln(std::string value);

    void logError(char message[], int error);
};

#endif