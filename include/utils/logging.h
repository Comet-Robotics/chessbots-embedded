#pragma once

#include <string>

enum DebugLevel {
    NONE,
    INFO,
    DEBUG,
    TRACE,
    RIDICULOUS, // Use if insane
};

#define SERIAL_CLEAR "\033[3J\033[H\033[2J"
#define SERIAL_WHITE "\e[0m"
#define SERIAL_RED "\e[31m"

void serial_printf(enum DebugLevel level, const char* fmt, ...);