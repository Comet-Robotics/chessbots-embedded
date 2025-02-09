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

// 
void confirmWiFi() {
    if (checkWiFiConnection()) {
        setWiFiConnectionStatus(true);
        wifiConnecting = false;
        logln((char*)"Connected to WiFi Network!", 2);

        // Connect to the server
        connectServer();
    } else {
        setWiFiConnectionStatus(false);
        wifiConnecting = true;
        log((char*)"Failed To Connect To WiFi: ", 2);
        log(getWifiStatus(WiFi.status()), 2);
        logln((char*)". Retrying... ", 2);

        // Check connection again after half a second
        timerDelay(500, &confirmWiFi);
    }
}

// Connects to a WiFi network with the SSID and Password set in env.h
void connectWiFI() {
    wifiConnecting = true;
    logln((char*)"Connecting to WiFi Network...", 2);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    // We delay the connection test to give the ESP time to fully connect to the network
    timerDelay(500, &confirmWiFi);
}

// Connects to a WiFi network with the SSID and Password set in env.h
void disconnectWiFI() {
    setWiFiConnectionStatus(false);
    WiFi.disconnect();
    logln((char*)"Disconnected From WiFI!", 2);
}

// Connects to a WiFi network with the SSID and Password set in env.h
void reconnectWiFI() {
    if (!wifiConnecting) {
        setWiFiConnectionStatus(false);
        logln((char*)"Disconnected From WiFI! Reconnecting...", 2);
        connectWiFI();
    }
}

// Creates a WiFi network with the SSID and Password set in env.h
void createWiFi() {
    logln((char*)"Creating Access Point", 2);
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
    logln((char*)"Access Point Created", 2);
    connectServer();
}

#endif