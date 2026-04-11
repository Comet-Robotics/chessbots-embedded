
#include <Arduino.h>

#include "../env.h"
#include "utils/logging.h"

// Prints a value or message through Serial. (The console)
// ln means it sends a new newline character

void serial_printf(enum DebugLevel level, const char* fmt, ...) {
    char buf[1024];
    
    if (level > LOGGING_LEVEL) {
        return;
    }

    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    Serial.print(buf);
}

void serial_clear() {
    serial_printf(DebugLevel::NONE, "\033[3J\033[H\033[2J");
}