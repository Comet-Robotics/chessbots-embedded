#ifndef CHESSBOT_STATUS_H
#define CHESSBOT_STATUS_H

bool getServerConnectionStatus();
void setServerConnectionStatus(bool value);

bool getWiFiConnectionStatus();
void setWiFiConnectionStatus(bool value);

bool getStoppedStatus();
void setStoppedStatus(bool value);

#endif