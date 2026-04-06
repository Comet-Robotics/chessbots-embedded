#include "utils/status.h"

bool wifiConnected = false;
bool serverConnected = false;
bool botStopped = false;

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