#ifndef CHESSBOT_STATUS_CPP
#define CHESSBOT_STATUS_CPP

// Associated Header File
#include "utils/status.h"

bool wifiConnected = false;
bool serverConnected = false;

bool getWiFiConnectionStatus() {
    return wifiConnected;
}
void setWiFiConnectionStatus(bool value) {
    wifiConnected = value;
}

bool getServerConnectionStatus() {
    return serverConnected;
}
void setServerConnectionStatus(bool value) {
    serverConnected = value;
}

#endif