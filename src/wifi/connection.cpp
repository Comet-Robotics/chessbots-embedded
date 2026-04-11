#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>

#include "wifi/connection.h"

#include "../../env.h"
#include "robot/robot.h"
#include "utils/logging.h"
#include "wifi/packet.h"

JsonDocument recv_packet(WiFiClient& client) {
    int index = 0;
    char raw_packet[500];
    JsonDocument packet;
    
    while (client.available()) {
        char data = client.read();

        if (data == ';' || index > 499) {
            break;
        } else {
            raw_packet[index] = data;
            index++;
        }
    }
        
    deserializeJson(packet, raw_packet);

    return packet;
}

// Sends a packet to the server
void send_packet(WiFiClient& client, JsonDocument packet) {
    // This takes that JSON object and sends it through the client's socket
    serializeJson(packet, client);

    // Sends a delimiter character to mark the end of the packet
    client.write(';');
}

void send_ping() {
    JsonDocument packet;
    packet["type"] = PING_RESPONSE;
    packet["batteryLevel"] = Robot::batteryLevel();

    send_packet(packet);
}