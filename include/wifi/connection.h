#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>

JsonDocument recv_packet(WiFiClient& client);
void send_packet(JsonDocument packet);
void send_ping();