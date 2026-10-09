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
    if (type == "SERVER_HELLO") {
        return SERVER_HELLO;
    }
    if (type == "SET_ABSOLUTE") {
        return SET_ABSOLUTE;
    }
    if (type == "TURN_BY_ANGLE") {
        return TURN_BY_ANGLE;
    }
    if (type == "DRIVE_ABSOLUTE") {
        return DRIVE_ABSOLUTE;
    }
    if (type == "DRIVE_TILES") {
        return DRIVE_TILES;
    }
    if (type == "CENTER_SEND") {
        return CENTER_SEND;
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

    } else if (type == SET_ABSOLUTE) {
        ASSERT_FIELD(packet, "x", double)
        ASSERT_FIELD(packet, "y", double)
        ASSERT_FIELD(packet, "rot", double)

        Coordinate2D coordinate = Coordinate2D(
            packet["x"].as<double>(),
            packet["y"].as<double>()
        );

        double rotation = packet["rot"].as<double>();

        r.set_position(coordinate);
        r.set_rotation(rotation);
    } else if (type == DRIVE_ABSOLUTE) {
        ASSERT_FIELD(packet, "x", double)
        ASSERT_FIELD(packet, "y", double)
        ASSERT_FIELD(packet, "rot", double)
        ASSERT_FIELD(packet, "packetId", const char *)


    } else if (type == TURN_BY_ANGLE) {
        ASSERT_FIELD(packet, "deltaHeadingRadians", double)
        ASSERT_FIELD(packet, "packetId", const char *)

        double delta_angle = packet["deltaHeadingRadians"].as<double>();

        r.turn(delta_angle, packet["packetId"].as<std::string>());

    } else if (type == DRIVE_TILES) {
        ASSERT_FIELD(packet, "tileDistance", double)
        ASSERT_FIELD(packet, "packetId", const char *)

        double tiles = packet["tileDistance"].as<double>();

        r.drive(tiles, packet["packetId"].as<std::string>());

    } else if (type == CENTER_SEND) {
        ASSERT_FIELD(packet, "packetId", const char *)
        r.center(packet["packetId"].as<std::string>());
    } else if (type == PING_SEND) {
        send_ping();
    }

    return true;
}