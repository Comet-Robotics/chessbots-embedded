#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>

#include "wifi/connection.h"

#include "../../env.h"
#include "robot/control/robot.h"
#include "utils/logging.h"
#include "utils/status.h"
#include "utils/timer.h"
#include "wifi/packet.h"

bool serverConnecting = false;
bool pinging = false;
int missedPings = 0;
unsigned long pingTimeoutTimer;

WiFiClient client;

// Called to connect to the server whose info is stored in env.h
void connectServer() {
    serial_printf(DebugLevel::DEBUG, "Connecting to Server...\n");
    if (client.connect(SERVER_IP, SERVER_PORT)) {
        // If successful, sets the connection status and stops trying to connect to the server
        setServerConnectionStatus(true);
        serverConnecting = false;
        serial_printf(DebugLevel::DEBUG, "Connected to Server!\n");

        // A handshake is an initial exchange of information, and a confirmation of a connection
        if (DO_HANDSHAKE) { initiateHandshake(); }
    } else {
        serverConnecting = true;
        // If unsuccessful, tries again in 5 seconds
        serial_printf(DebugLevel::DEBUG, "Connection To Server Failed! Retrying...\n");

        timerDelay(HANDSHAKE_INTERVAL, &connectServer);
    }
}

// Completely disconnect from the server
void disconnectServer() {
    setServerConnectionStatus(false);
    client.stop();
    serial_printf(DebugLevel::DEBUG, "Disconnected From Server!\n");
}

// If not connected to the server (whether by disconnect or by lost connection), reconnects
void reconnectServer() {
    if (!serverConnecting) {
        setServerConnectionStatus(false);
        serial_printf(DebugLevel::DEBUG, "Disconnected From Server! Reconnecting...\n");
        connectServer();
    }
}

// Checks whether still connected to server
bool checkServerConnection() {
    return client.connected();
}

// Sends an initial packet to the server. Contains the mac address of this bot
void initiateHandshake() {
    JsonDocument packet;
    constructHelloPacket(packet);
    serial_printf(DebugLevel::DEBUG, "Sending Handshake...\n");
    sendPacket(packet);
}

// The buffer size is 500 characters. If there are issues right after
// accepting a packet, the buffer size may be the culprit
JsonDocument acceptData() {
    JsonDocument packet;
    if (client.available()) {
        // Allocates a buffer to hold the incoming packet
        char rawPacket[500];
        int len = 0;
        bool packetDone = false;
        while (client.available() || !packetDone) {
            // Reads in a single character
            char data = client.read();
            if (data == ';' || len > 499) {
                // If the delimiter character is encountered, the packet is done
                packetDone = true;
            } else {
                // Adds the character to the buffer
                rawPacket[len] = data;
                len++;
            }
        }

        // The packet is in the form of a JSON. We use a library to handle them
        
        // This turns the character buffer into a fully formed JSON object
        deserializeJson(packet, rawPacket);
    }

    return packet;
}

// Sends a packet to the server
void sendPacket(JsonDocument& packet) {
    // This takes that JSON object and sends it through the client's socket
    serializeJson(packet, client);
    // Sends a delimiter character to mark the end of the packet
    client.write(';');
}

void sendActionSuccess(std::string messageId) {
    JsonDocument packet;
    constructSuccessPacket(packet, messageId);
    serial_printf(DebugLevel::DEBUG, "Sending Action Success...\n");
    sendPacket(packet);
}

void sendActionFail(std::string messageId) {
    JsonDocument packet;
    constructFailPacket(packet, messageId);
    serial_printf(DebugLevel::DEBUG, "Sending Action Success...\n");
    sendPacket(packet);
}

void pingTimeout() {
    if (DO_PINGING) {
        missedPings++;
        serial_printf(DebugLevel::DEBUG, "%u missed pings!\n", missedPings);

        if (missedPings >= PING_MAX_MISSES) {
            serial_printf(DebugLevel::DEBUG, "SERVER TIMED OUT!\n");
            // TODO: stop();
            pinging = false;
        } else {
            pingTimeoutTimer = timerDelay(PING_TIMEOUT, &pingTimeout);
        }
    }
}

void sendPingResponse() {
    if (DO_PINGING) {
        JsonDocument packet;
        constructPingPacket(packet);
        serial_printf(DebugLevel::DEBUG, "Sending Ping Response...\n");
        sendPacket(packet);
        if (pinging) {
            timerReset(pingTimeoutTimer);
        } else {
            pingTimeoutTimer = timerDelay(PING_TIMEOUT, &pingTimeout);
            serial_printf(DebugLevel::DEBUG, "Started Ping Timeout Timer\n");
            pinging = true;
        }
        missedPings = 0;
    }
}