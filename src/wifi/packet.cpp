#ifndef CHESSBOT_PACKET_CPP
#define CHESSBOT_PACKET_CPP

#include "wifi/packet.h"

#include "Arduino.h"
#include <ArduinoJson.h>
#include "esp_mac.h"
#include <sstream>
#include <iostream>

#include "utils/logging.h"
#include "utils/status.h"
#include "robot/control.h"

namespace ChessBot
{
    // These are the various different supported message types that can be sent over TCP
    const char* CLIENT_HELLO = "CLIENT_HELLO";
    const char* SERVER_HELLO = "SERVER_HELLO";
    const char* PING_SEND = "PING_SEND";
    const char* PING_RESPONSE = "PING_RESPONSE";
    const char* QUERY_VAR = "QUERY_VAR";
    const char* QUERY_RESPONSE = "QUERY_RESPONSE";
    const char* SET_VAR = "SET_VAR";
    const char* TURN_BY_ANGLE = "TURN_BY_ANGLE";
    const char* DRIVE_TILES = "DRIVE_TILES";
    const char* ACTION_SUCCESS = "ACTION_SUCCESS";
    const char* ACTION_FAIL = "ACTION_FAIL";
    const char* DRIVE_TANK = "DRIVE_TANK";
    const char* ESTOP = "ESTOP";

    void handlePacket(JsonDocument& packet) {
        if (packet["type"] == SERVER_HELLO) {
        } else if (packet["type"] == DRIVE_TANK) {
            drive(packet["left"], packet["right"]);
        }
    }

    std::string unint8ArrayToHexString(uint8_t* oldArray, int len) {
        std::string result;
        result.reserve(len * 2);
        static constexpr char hex[] = "0123456789abcdef";

        for (int i = 0; i < len; i++) {
            if (i != 0) result.push_back(':');
            result.push_back(hex[oldArray[i] / 16]);
            result.push_back(hex[oldArray[i] % 16]);
        }
        logln(result);
        return result;
    }

    void constructHelloPacket(JsonDocument& packet) {
        packet["type"] = CLIENT_HELLO;
        uint8_t mac[8];
        esp_efuse_mac_get_default(mac);
        std::string stringMac = unint8ArrayToHexString(mac, 6);
        packet["macAddress"] = stringMac;
    }

    void constructSuccessPacket(JsonDocument& packet) {}

    void constructFailPacket(JsonDocument& packet) {}
};

#endif