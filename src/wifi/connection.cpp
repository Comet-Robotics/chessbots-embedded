#ifndef CHESSBOT_CONNECTION_CPP
#define CHESSBOT_CONNECTION_CPP

#include "wifi/connection.h"

#include "Arduino.h"
#include <ArduinoJson.h>
#include "utils/logging.h"
#include "utils/timer.h"
#include "utils/status.h"
#include "wifi/packet.h"
#include "../env.h"

#include "WiFi.h"

namespace ChessBot
{
    WiFiClient client;
    bool connecting = false;
    
    void connect() {
        logln((char*)"Connecting to Server...");
        if (!client.connect(SERVER_IP, SERVER_PORT)) {
            logln((char*)"Connection Failed! Retrying...");
            connecting = true;
            timerDelay(5000, &connect);
        } else {
            setServerConnectionStatus(true);
            connecting = false;
            logln((char*)"Connected to Server!");
            if (DO_HANDSHAKE) {
                initiateHandshake();
            }
        }
    }

    void disconnect() {
        setServerConnectionStatus(false);
        client.stop();
    }

    void reconnect() {
        if (!connecting) {
            setServerConnectionStatus(false);
            logln((char*)"Disconnected! Reconnecting...");
            connect();
        }
    }

    bool testConnection() {
        return client.connected();
    }

    void initiateHandshake() {
        JsonDocument packet;
        constructHelloPacket(packet);
        logln((char*)"Sending Handshake...");
        sendPacket(packet);
    }

    // The buffer size is 500 characters. If there are issues right after
    // accepting a packet, the buffer size may be the culprit
    void acceptData() {
        if (client.available()) {
            char rawPacket[500];
            int len = 0;
            bool packetDone = false;
            while (client.available() || !packetDone) {
                char data = client.read();
                if (data == ';') {
                    packetDone = true;
                } else {
                    rawPacket[len] = data;
                    len++;
                }
            }
            JsonDocument packet;
            deserializeJson(packet, rawPacket);
            log((char*)"Received Packet: ");
            serializeJson(packet, Serial);
            logln((char*)"");
            handlePacket(packet);
        }
    }

    void sendPacket(JsonDocument& packet) {
        serializeJson(packet, client);
        client.write(';');
        logln((char*)"Sent Packet: ");
        serializeJson(packet, Serial);
        logln((char*)"");
    }
};

#endif