#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>

#include "wifi/connection.h"

#include "../../env.h"
#include "robot/robot.h"
#include "utils/logging.h"
#include "wifi/packet.h"

#if defined(ONLINE) \
    && (!defined(WIFI_SSID) || !defined(WIFI_PASSWORD) || !defined(SERVER_IP) || !defined(SERVER_PORT))
    #error ONLINE defined but one of (WIFI_SSID, WIFI_PASSWORD, SERVER_IP, SERVER_PORT) is not set
#endif

WiFiClient client;
u_int32_t last_connection_try_time;

inline bool connected() {
    return WiFi.status() == WL_CONNECTED && client.connected();
}

void connection_check_reconnect() {
    u_int32_t delta_con_time = millis() - last_connection_try_time;
    if (WiFi.status() != WL_CONNECTED && delta_con_time > 5000) {
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        last_connection_try_time = millis();
    }

    if (!client.connected()) {
        if (client.connect(SERVER_IP, SERVER_PORT)) {
            send_handshake();
        }
    }
}

std::optional<JsonDocument> recv_packet() {
    if (!connected()) {
        return std::nullopt;
    }

    int index = 0;
    char raw_packet[500];
    JsonDocument packet;
    
    // Tries to read the whole packet in one go, might break if the underlying TCP packet is fragmented
    while (client.available()) {
        char data = client.read();

        if (data == ';' || index > 499) {
            break;
        } else {
            raw_packet[index] = data;
            index++;
        }
    }

    if (deserializeJson(packet, raw_packet) != DeserializationError::Ok) {
        return std::nullopt;
    }

    return std::make_optional(packet);
}

// Sends a packet to the server
void send_packet(JsonDocument packet) {
    serializeJson(packet, client);
    client.write(';');
}

void send_handshake() {
    uint8_t mac[8];
    JsonDocument packet;    

    esp_efuse_mac_get_default(mac);
    auto stringMac = unint8ArrayToHexString(mac, 6);

    packet["type"] = "CLIENT_HELLO";
    packet["macAddress"] = stringMac;

    send_packet(packet);
}

void send_success(std::string id) {
    JsonDocument packet;
    packet["type"] = "ACTION_SUCCESS";
    packet["packetId"] = id;

    send_packet(packet);
}

void send_ping() {
    JsonDocument packet;
    packet["type"] = "PING_RESPONSE";
    packet["batteryLevel"] = Robot::batteryLevel();

    send_packet(packet);
}