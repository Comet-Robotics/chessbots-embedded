#ifndef CHESSBOT_WIRELESS_CPP
#define CHESSBOT_WIRELESS_CPP

#include "wifi/wireless.h"

#include "Arduino.h"
#include "WiFi.h"

#include "utils/logging.h"
#include "utils/timer.h"
#include "utils/status.h"
#include "wifi/connection.h"
#include "../env.h"

namespace ChessBot
{
    // Gets the status of the WiFi connection. Called whenever connecting to WiFi fails to get more info
    char* getWifiStatus(int status){
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
    void checkConnection() {
        if (WiFi.status() == WL_CONNECTED) {
            setWiFiConnectionStatus(true);

            logln((char*)"Connected to WiFi Network!", 2);
            // Connect to the server
            connect();
        } else {
            setWiFiConnectionStatus(false);

            log((char*)"Connection Failed: ", 2);
            log(getWifiStatus(WiFi.status()), 2);
            logln((char*)". Retrying... ", 2);

            // Check connection again after half a second
            timerDelay(500, &checkConnection);
        }
    }

    // Connects to a WiFi network with the SSID and Password set in env.h
    void connectWiFI() {
        logln((char*)"Connecting to WiFi Network...", 2);
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        timerDelay(500, &checkConnection);
    }

    // Creates and access point (WiFi network) with the SSID and Password set in env.h
    void createWiFi() {
        logln((char*)"Creating Access Point", 2);
        WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
        logln((char*)"Access Point Created", 2);
        connect();
    }
};

#endif