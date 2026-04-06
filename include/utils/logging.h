#pragma once

#include <string>

enum DebugLevel {
    NONE,
    INFO,
    DEBUG,
    TRACE,
    RIDICULOUS, // Use if insane
};

void serial_printf(enum DebugLevel level, const char* fmt, ...);
void serial_clear();