#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

#include "robot/robot.h"

enum PacketType : uint8_t {
    CLIENT_HELLO,
    SERVER_HELLO,
    PING_SEND,
    PING_RESPONSE,
    QUERY_VAR,
    QUERY_RESPONSE,
    SET_VAR,
    TURN_BY_ANGLE,
    DRIVE_TILES,
    DRIVE_TICKS,
    ACTION_SUCCESS,
    ACTION_FAIL,
    DRIVE_TANK,
    ESTOP,
    CUBIC,
    QUADRATIC,
    SPIN_RADIANS,
    BS_MOVE,
    CENTER_SEND,

    ERROR // Not a valid packet type
};

// Checks that the json document has the correct field and type
// If it doesn't contain the correct field, returns false
#define ASSERT_FIELD(packet, key, _type) \
  if (!packet[key].is<_type>()) { \
    serial_printf(DebugLevel::NONE, "Received a packet that did not contain field %s\n", key); \
    return false; \
  }

PacketType parse_packet_type(const char * type);
bool handle_packet(Robot& r, JsonDocument packet);
std::string unint8ArrayToHexString(uint8_t* oldArray, int len);
