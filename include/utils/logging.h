#ifndef CHESSBOT_LOGGING_H
#define CHESSBOT_LOGGING_H

// Built-In Libraries
#include <string>

void serialLog(char message[], int serialLoggingLevel);
void serialLog(int value, int serialLoggingLevel);
void serialLog(float value, int serialLoggingLevel);

void serialLogln(char message[], int serialLoggingLevel);
void serialLogln(int value, int serialLoggingLevel);
void serialLogln(float value, int serialLoggingLevel);
void serialLogln(std::string value, int serialLoggingLevel);

void serialLogError(char message[], int error);

#endif