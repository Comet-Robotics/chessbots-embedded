#ifndef CHESSBOT_WIRELESS_CPP
#define CHESSBOT_WIRELESS_CPP

#include "wifi/wireless.h"

#include "Arduino.h"
#include "utils/logging.h"
#include "utils/timer.h"
#include "utils/status.h"
#include "wifi/connection.h"
#include "../env.h"

#include "WiFi.h" // Built in library

namespace ChessBot
{
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

    void checkConnection() {
        if (WiFi.status() == WL_CONNECTED) {
            setWiFiConnectionStatus(true);
            logln((char*)"Connected to WiFi Network!");
            // Connect to the PC server
            connect();
        } else {
            setWiFiConnectionStatus(false);
            log((char*)"Connection Failed: ");
            log(getWifiStatus(WiFi.status()));
            logln((char*)". Retrying... ");
            timerDelay(500, &checkConnection);
        }
    }

    void connectWiFI() {
        logln((char*)"Connecting to WiFi Network...");
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        timerDelay(500, &checkConnection);
    }

    void createWiFi() {
        logln((char*)"Creating Access Point");
        WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
        logln((char*)"Access Point Created");
        connect();
    }
};

#endif