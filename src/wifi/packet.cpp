#include <Arduino.h>
#include <ArduinoJson.h>
#include <esp_mac.h>
#include <string>

#include "wifi/packet.h"

#include "robot/robot.h"
#include "utils/config.h"
#include "utils/functions.h"
#include "utils/logging.h"
#include "wifi/connection.h"

PacketType parse_packet_type(std::string type) {
    if (type == "CLIENT_HELLO") {
        return CLIENT_HELLO;
    }
    if (type == "SERVER_HELLO") {
        return SERVER_HELLO;
    }
    if (type == "PING_SEND") {
        return PING_SEND;
    }
    if (type == "PING_RESPONSE") {
        return PING_RESPONSE;
    }
    if (type == "TURN_BY_ANGLE") {
        return TURN_BY_ANGLE;
    }
    if (type == "DRIVE_TILES") {
        return DRIVE_TILES;
    }
    if (type == "DRIVE_TICKS") {
        return DRIVE_TICKS;
    }
    if (type == "ACTION_SUCCESS") {
        return ACTION_SUCCESS;
    }
    if (type == "ACTION_FAIL") {
        return ACTION_FAIL;
    }
    if (type == "DRIVE_TANK") {
        return DRIVE_TANK;
    }
    if (type == "ESTOP") {
        return ESTOP;
    }
    if (type == "SPIN_RADIANS") {
        return SPIN_RADIANS;
    }
    if (type == "BS_MOVE") {
        return BS_MOVE;
    }

    return ERROR;
}

// Takes a packet a does specific things based on the type
bool handle_packet(Robot& r, JsonDocument packet) {
    ASSERT_FIELD(packet, "type", const char *)

    PacketType type = parse_packet_type(packet["type"].as<std::string>());
    serial_printf(DebugLevel::INFO, "Received a packet of type %d\n", type);

    if (type == ERROR) {
        return false;
    }

    if (type == SERVER_HELLO) {
        // When we initiate a handshake, the server sends a handshake back. This server handshake
        // contains any variable that should be changed in this bot's config
        ASSERT_FIELD(packet, "config", JsonObject)

        setConfig(packet["config"].as<JsonObject>());

    } else if (type == DRIVE_TANK) {
        // Manual control of the robot

        ASSERT_FIELD(packet, "left", double)
        ASSERT_FIELD(packet, "left", double)
        ASSERT_FIELD(packet, "packetId", const char *)

        std::tuple<double, double> power = std::make_tuple(
            packet["left"].as<double>(), packet["right"].as<double>()
        );

        r.drive(power, packet["packetId"].as<std::string>());

    } else if (type == ESTOP) {
        r.stop();

    } else if (type == TURN_BY_ANGLE) {
        ASSERT_FIELD(packet, "deltaHeadingRadians", double)
        ASSERT_FIELD(packet, "packetId", const char *)

        double delta_angle = packet["deltaHeadingRadians"].as<double>();
        const char *id = packet["packetId"];

        r.turn(delta_angle, packet["packetId"].as<std::string>());

    } else if (type == DRIVE_TILES) {
        ASSERT_FIELD(packet, "tileDistance", double)
        ASSERT_FIELD(packet, "packetId", const char *)

        double tiles = packet["tileDistance"].as<double>();

        r.drive(tiles, packet["packetId"].as<std::string>());

    } else if (type == PING_SEND) {
        send_ping();
    }

    return true;
}