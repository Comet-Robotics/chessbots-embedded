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
char* getWifiStatus(int status) {
    switch(status){
        case WL_IDLE_STATUS:
        return (char*)"WL_IDLE_STATUS";
        case WL_SCAN_COMPLETED:
        return (char*)"WL_SCAN_COMPLETED";
        case WL_NO_SSID_AVAIL:
        return (char*)"WL_NO_SSID_AVAIL";
        case WL_CONNECT_FAILED:
        return (char*)"WL_CONNECT_FAILED";
        case WL_CONNECTION_LOST:
        return (char*)"WL_CONNECTION_LOST";
        case WL_CONNECTED:
        return (char*)"WL_CONNECTED";
        case WL_DISCONNECTED:
        return (char*)"WL_DISCONNECTED";
    }
    return (char*)"No Status";
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
        serialLogln((char*)"Connected to WiFi Network!", 2);

        // Connect to the server
        connectServer();
    } else {
        setWiFiConnectionStatus(false);
        wifiConnecting = true;
        serialLog((char*)"Failed To Connect To WiFi: ", 2);
        serialLog(getWifiStatus(WiFi.status()), 2);
        serialLogln((char*)". Retrying... ", 2);

        // Check connection again after half a second
        timerDelay(500, &confirmWiFi);
    }
}

// Connects to a WiFi network with the SSID and Password set in env.h
void connectWiFI() {
    wifiConnecting = true;
    serialLogln((char*)"Connecting to WiFi Network...", 2);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    // We delay the connection test to give the ESP time to fully connect to the network
    timerDelay(500, &confirmWiFi);
}

// Completely disconnect from WiFi
void disconnectWiFI() {
    setWiFiConnectionStatus(false);
    WiFi.disconnect();
    serialLogln((char*)"Disconnected From WiFI!", 2);
}

// If not connected to WiFi (whether by disconnect or by lost connection), reconnects
void reconnectWiFI() {
    if (!wifiConnecting) {
        setWiFiConnectionStatus(false);
        serialLogln((char*)"Disconnected From WiFI! Reconnecting...", 2);
        connectWiFI();
    }
}

// Creates a WiFi network with the SSID and Password set in env.h
void createWiFi() {
    serialLogln((char*)"Creating Access Point", 2);
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
    serialLogln((char*)"Access Point Created", 2);
    connectServer();
}

#endif