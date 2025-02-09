#ifndef CHESSBOT_CONNECTION_H
#define CHESSBOT_CONNECTION_H

#include "Arduino.h"
#include <ArduinoJson.h>

void connect();
void disconnect();
void reconnect();
bool testConnection();
void initiateHandshake();

void acceptData();
void sendPacket(JsonDocument& packet);

#endif