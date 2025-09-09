#ifndef CHESSBOT_CONNECTION_CPP
#define CHESSBOT_CONNECTION_CPP

// Associated Header File
#include "wifi/connection.h"

// Built-In Libraries
#include "Arduino.h"
#include "WiFi.h"

// External Libraries
#include <ArduinoJson.h>

// Custom Libraries
#include "wifi/packet.h"
#include "utils/logging.h"
#include "utils/timer.h"
#include "utils/status.h"
#include "robot/control.h"

#include "../env.h"

bool serverConnecting = false;
bool pinging = false;
int missedPings = 0;
unsigned long pingTimeoutTimer;

WiFiClient client;

// Called to connect to the server whose info is stored in env.h
void connectServer() {
    serialLogln("Connecting to Server...", 2);
    if (client.connect(SERVER_IP, SERVER_PORT)) {
        // If successful, sets the connection status and stops trying to connect to the server
        setServerConnectionStatus(true);
        serverConnecting = false;
        serialLogln("Connected to Server!", 2);

        // A handshake is an initial exchange of information, and a confirmation of a connection
        if (DO_HANDSHAKE) { initiateHandshake(); }
    } else {
        serverConnecting = true;
        // If unsuccessful, tries again in 5 seconds
        serialLogln("Connection To Server Failed! Retrying...", 2);

        timerDelay(HANDSHAKE_INTERVAL, &connectServer);
    }
}

// Completely disconnect from the server
void disconnectServer() {
    setServerConnectionStatus(false);
    client.stop();
    serialLogln("Disconnected From Server!", 2);
}

// If not connected to the server (whether by disconnect or by lost connection), reconnects
void reconnectServer() {
    if (!serverConnecting) {
        setServerConnectionStatus(false);
        serialLogln("Disconnected From Server! Reconnecting...", 2);
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
    serialLogln((char*)"Sending Handshake...", 2);
    sendPacket(packet);
}

// The buffer size is 500 characters. If there are issues right after
// accepting a packet, the buffer size may be the culprit
void acceptData() {
    if (client.available()) {
        // Allocates a buffer to hold the incoming packet
        char rawPacket[500];
        int len = 0;
        bool packetDone = false;
        while (client.available() || !packetDone) {
            // Reads in a single character
            char data = client.read();
            if (data == ';') {
                // If the delimiter character is encountered, the packet is done
                packetDone = true;
            } else {
                // Adds the character to the buffer
                rawPacket[len] = data;
                len++;
            }
        }

        // The packet is in the form of a JSON. We use a library to handle them
        JsonDocument packet;
        // This turns the character buffer into a fully formed JSON object
        deserializeJson(packet, rawPacket);
        serialLog("Received Packet: ", 2);
        // This takes that JSON object and prints it to Serial (the console) for debugging purposes
        if (LOGGING_LEVEL >= 3) serializeJson(packet, Serial);
        serialLog("\n", 2);

        // Actually does something with the received packet
        handlePacket(packet);
    }
}

// Sends a packet to the server
void sendPacket(JsonDocument& packet) {
    // This takes that JSON object and sends it through the client's socket
    serializeJson(packet, client);
    // Sends a delimiter character to mark the end of the packet
    client.write(';');
    serialLogln("Sent Packet: ", 2);
    // This takes that JSON object and prints it to Serial (the console) for debugging purposes
    if (LOGGING_LEVEL >= 3) serializeJson(packet, Serial);
    serialLog("\n", 2);
}

void sendActionSuccess(std::string messageId) {
    JsonDocument packet;
    constructSuccessPacket(packet, messageId);
    serialLogln((char*)"Sending Action Success...", 2);
    sendPacket(packet);
}

void sendActionFail(std::string messageId) {
    JsonDocument packet;
    constructFailPacket(packet, messageId);
    serialLogln((char*)"Sending Action Success...", 2);
    sendPacket(packet);
}

void pingTimeout() {
    missedPings++;
    serialLog(missedPings, 2);
    serialLogln((char*)" missed ping!", 2);
    if (missedPings >= PING_MAX_MISSES) {
        serialLogln((char*)"SERVER TIMED OUT!", 2);
        stop();
    }
}

void sendPingResponse() {
    JsonDocument packet;
    constructPingPacket(packet);
    serialLogln((char*)"Sending Ping Response...", 2);
    sendPacket(packet);
    if (pinging) {
        timerReset(pingTimeoutTimer);
        missedPings = 0;
    } else {
        pingTimeoutTimer = timerDelay(PING_TIMEOUT, &pingTimeout);
        serialLogln((char*)"Started Ping Timeout Timer", 2);
        missedPings = 0;
    }
}

#endif