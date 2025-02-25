#ifndef CHESSBOT_PACKET_CPP
#define CHESSBOT_PACKET_CPP

// Associated Header File
#include "wifi/packet.h"

// Built-In Libraries
#include "Arduino.h"
#include "esp_mac.h"

// External Libraries
#include <ArduinoJson.h>

// Custom Libraries
#include "utils/logging.h"
#include "utils/status.h"
#include "utils/functions.h"
#include "utils/config.h"
#include "robot/control.h"
#include "wifi/connection.h"

// These are the various different supported message types that can be sent over TCP

//Instead of setting constant chars, we'll have the user input their tcp name.
// const char* CLIENT_HELLO = "CLIENT_HELLO";
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
// const char* SUCCESS_MESSAGE = "SUCCESS_MESSAGE";
// const char* ERR_MESSAGE = "ERR_MESSAGE";

// Takes a packet a does specific things based on the type
void handlePacket(JsonDocument packet) {
    // Sadly a switch case can't be used due to the packet type being a string.
    // We do this to allow the packets to be more readable when logged
    if (packet["type"] == SERVER_HELLO) {
        // When we initiate a handshake, the server sends a handshake back. This server handshake
        // contains any variable that should be changed in this bot's config
        setConfig(packet["config"].as<JsonObject>());
    } else if (packet["type"] == DRIVE_TANK) {
        // This is received when the bot is being manually controlled via the debug page
        drive(packet["left"], packet["right"]);
    }
}

// This creates the handshake packet sent to the server when this bot connects to it
void constructPacket(JsonDocument& packet, std::string packetType) {
    packet["type"] = packetType;
    uint8_t mac[8];
    // Gets the mac address of this esp
    esp_efuse_mac_get_default(mac);
    // Converts the mac address into the form the server is expecting
    std::string stringMac = unint8ArrayToHexString(mac, 6);
    packet["macAddress"] = stringMac;
}

#endif