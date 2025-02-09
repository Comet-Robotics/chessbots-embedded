#ifndef CHESSBOT_WIRELESS_H
#define CHESSBOT_WIRELESS_H

char* getWifiStatus(int status);
bool checkWiFiConnection();
void confirmWiFi();
void connectWiFI();
void reconnectWiFI();
void disconnectWiFI();
void createWiFi();

#endif