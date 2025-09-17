#ifndef CHESSBOT_PACKET_H
#define CHESSBOT_PACKET_H

#include "Arduino.h"
#include <ArduinoJson.h>


void handlePacket(JsonDocument packet);
std::string unint8ArrayToHexString(uint8_t* oldArray, int len);

void constructHelloPacket(JsonDocument& packet);
void constructSuccessPacket(JsonDocument& packet, std::string messageId);
void constructFailPacket(JsonDocument& packet, std::string messageId);
void constructPingPacket(JsonDocument& packet);
#endif