#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>

#include <optional>

extern WiFiClient client;

bool connected();
bool reconnect();
std::optional<JsonDocument> recv_packet();

void send_packet(JsonDocument packet);
void send_handshake();
void send_success(std::string id);
void send_ping();