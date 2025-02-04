#ifndef CHESSBOT_STATUS_CPP
#define CHESSBOT_STATUS_CPP

#include "utils/status.h"

namespace ChessBot
{
    bool wifiConnected = false;
    bool serverConnected = false;
    bool doLogging = true;

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

    bool getLoggingStatus() {
        return doLogging;
    }
    void setLoggingStatus(bool value) {
        doLogging = value;
    }
};

#endif