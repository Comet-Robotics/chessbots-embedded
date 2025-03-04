#ifndef CHESSBOT_PACKET_H
#define CHESSBOT_PACKET_H

#include "Arduino.h"
#include <ArduinoJson.h>


void handlePacket(JsonDocument packet);
std::string unint8ArrayToHexString(uint8_t* oldArray, int len);

void constructPacket(JsonDocument& packet, std::string packetType, std::string messageId);
#endif