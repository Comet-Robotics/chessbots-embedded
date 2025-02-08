#ifndef CHESSBOT_FUNCTIONS_CPP
#define CHESSBOT_FUNCTIONS_CPP

// Associated Header File
#include "utils/functions.h"

// Built-In Libraries
#include "Arduino.h"

// Custom Libraries
#include "utils/logging.h"

namespace ChessBot
{
    // This custom map function takes a number, and scales it from one range to another.
    // We use this to change a number between 0-1 to a number between 0-255
    float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    // When we get the mac address from the esp, it gives it as an array of unsigned
    // 8 bit integers. The server is expecting it as a hex string separated by colons
    std::string unint8ArrayToHexString(uint8_t* oldArray, int len) {
        std::string result;
        // Each int ends up as a two character hex, so we allocate twice the length
        result.reserve(len * 2);
        // This is just a lookup array to help convert the int
        static constexpr char hex[] = "0123456789abcdef";

        // Convert each int into its corresponding two hex characters and add them into the
        // string separated by a colon
        for (int i = 0; i < len; i++) {
            if (i != 0) result.push_back(':');
            result.push_back(hex[oldArray[i] / 16]);
            result.push_back(hex[oldArray[i] % 16]);
        }
        logln(result, 4);
        return result;
    }
};

#endif