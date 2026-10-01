#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>

#include <optional>

extern WiFiClient client;

bool connected_wifi();
bool connected_server();
bool reconnect_wifi();
bool reconnect_server();

std::optional<JsonDocument> recv_packet();

void send_packet(JsonDocument packet);
void send_handshake();
void send_success(std::string id);
void send_ping();