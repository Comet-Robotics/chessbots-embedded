#ifndef CHESSBOT_CONNECTION_H
#define CHESSBOT_CONNECTION_H

#include "Arduino.h"
#include <ArduinoJson.h>

void connectServer();
void disconnectServer();
void reconnectServer();
bool checkServerConnection();
void initiateHandshake();

JsonDocument acceptData();
void sendPacket(JsonDocument& packet);
void sendActionSuccess(std::string messageId);
void sendActionFail(std::string messageId);
void sendPingResponse();

#endif