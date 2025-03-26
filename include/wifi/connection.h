#ifndef CHESSBOT_CONNECTION_H
#define CHESSBOT_CONNECTION_H

#include "Arduino.h"
#include <ArduinoJson.h>

void connectServer();
void disconnectServer();
void reconnectServer();
bool checkServerConnection();
void initiateHandshake();
void pingServer();

void acceptData();
void sendPacket(JsonDocument& packet);

#endif