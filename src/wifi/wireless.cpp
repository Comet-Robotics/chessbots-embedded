#ifndef CHESSBOT_WIRELESS_CPP
#define CHESSBOT_WIRELESS_CPP

// Associated Header File
#include "wifi/wireless.h"

// Built-In Libraries
#include "Arduino.h"
#include "WiFi.h"

// Custom Libraries
#include "utils/logging.h"
#include "utils/timer.h"
#include "utils/status.h"
#include "wifi/connection.h"
#include "../env.h"

bool wifiConnecting = false;

// Gets the status of the WiFi connection. Called whenever connecting to WiFi fails to get more info
const char* getWifiStatus(int status) {
    switch(status){
        case WL_IDLE_STATUS:
        return "WL_IDLE_STATUS";
        case WL_SCAN_COMPLETED:
        return "WL_SCAN_COMPLETED";
        case WL_NO_SSID_AVAIL:
        return "WL_NO_SSID_AVAIL";
        case WL_CONNECT_FAILED:
        return "WL_CONNECT_FAILED";
        case WL_CONNECTION_LOST:
        return "WL_CONNECTION_LOST";
        case WL_CONNECTED:
        return "WL_CONNECTED";
        case WL_DISCONNECTED:
        return "WL_DISCONNECTED";
    }
    return "No Status";
}

// Checks whether the bot is connected to a WiFi network
bool checkWiFiConnection() {
    return WiFi.status() == WL_CONNECTED;
}

// If not connected to WiFi by now, serialLog 'why'
// Tries to connect until success. After connected to WiFi, starts trying to connect to the server
void confirmWiFi() {
    if (checkWiFiConnection()) {
        setWiFiConnectionStatus(true);
        wifiConnecting = false;
        serialLogln("Connected to WiFi Network!", 2);

        // Connect to the server
        connectServer();
    } else {
        setWiFiConnectionStatus(false);
        wifiConnecting = true;
        serialLog("Failed To Connect To WiFi: ", 2);
        serialLog(getWifiStatus(WiFi.status()), 2);
        serialLogln(". Retrying... ", 2);

        // Check connection again after half a second
        timerDelay(500, &confirmWiFi);
    }
}

// Connects to a WiFi network with the SSID and Password set in env.h
void connectWiFI() {
    wifiConnecting = true;
    serialLogln("Connecting to WiFi Network...", 2);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    // We delay the connection test to give the ESP time to fully connect to the network
    timerDelay(500, &confirmWiFi);
}

// Completely disconnect from WiFi
void disconnectWiFI() {
    setWiFiConnectionStatus(false);
    WiFi.disconnect();
    serialLogln("Disconnected From WiFI!", 2);
}

// If not connected to WiFi (whether by disconnect or by lost connection), reconnects
void reconnectWiFI() {
    if (!wifiConnecting) {
        setWiFiConnectionStatus(false);
        serialLogln("Disconnected From WiFI! Reconnecting...", 2);
        connectWiFI();
    }
}

// Creates a WiFi network with the SSID and Password set in env.h
void createWiFi() {
    serialLogln("Creating Access Point", 2);
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
    serialLogln("Access Point Created", 2);
    connectServer();
}

#endif