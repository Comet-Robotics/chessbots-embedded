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

#include "../env.h"

bool serverConnecting = false;

WiFiClient client;

// Called to connect to the server whose info is stored in env.h
void connectServer() {
    logln((char*)"Connecting to Server...", 2);
    if (client.connect(SERVER_IP, SERVER_PORT)) 
    {
        // If successful, sets the connection status and stops trying to connect to the server
        setServerConnectionStatus(true);
        serverConnecting = false;
        logln((char*)"Connected to Server!", 2);

        // A handshake is an initial exchange of information, and a confirmation of a connection
        if (DO_HANDSHAKE) {
            createAndSendPacket(2, "hello", "Hello");
        }
    } 
    else 
    {
        serverConnecting = true;
        // If unsuccessful, tries again in 5 seconds
        logln((char*)"Connection To Server Failed! Retrying...", 2);

        timerDelay(5000, &connectServer);
    }
}

// Completely disconnect from the server
void disconnectServer() {
    setServerConnectionStatus(false);
    client.stop();
    logln((char*)"Disconnected From Server!", 2);
}

// If not connected to the server (whether by disconnect or by lost connection), reconnects
void reconnectServer() {
    if (!serverConnecting) {
        setServerConnectionStatus(false);
        logln((char*)"Disconnected From Server! Reconnecting...", 2);
        connectServer();
    }
}

// Checks whether still connected to server
bool checkServerConnection() {
    return client.connected();
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
        log((char*)"Received Packet: ", 2);
        // This takes that JSON object and prints it to Serial (the console) for debugging purposes
        if (LOGGING_LEVEL >= 3) serializeJson(packet, Serial);
        log((char*)"\n", 2);

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
    logln((char*)"Sent Packet: ", 2);
    // This takes that JSON object and prints it to Serial (the console) for debugging purposes
    if (LOGGING_LEVEL >= 3) serializeJson(packet, Serial);
    log((char*)"\n", 2);
}

//Additional method created to call the construction of the packet and then to send it.
void createAndSendPacket(uint8_t priority, std::string message, std::string messageId)
{
    JsonDocument packet;
    //How I have it right now is it calls the different packet messages according to the message. Might
    //be inefficient and require changing.
    if(message == "hello")
    {
        logln((char*)"Sending handshake...", priority);
        constructHelloPacket(packet);
    }
    else if(message == "success")
    {
        logln((char*)"Action succeeded!", priority);
        constructSuccessPacket(packet, messageId);
    }
    else if(message == "fail")
    {
        logln((char*)"Action failed!", priority);
        constructFailPacket(packet, messageId);
    }
    sendPacket(packet);
}

#endif