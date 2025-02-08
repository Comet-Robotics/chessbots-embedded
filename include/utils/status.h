#ifndef CHESSBOT_STATUS_H
#define CHESSBOT_STATUS_H

namespace ChessBot
{
    bool getServerConnectionStatus();
    void setServerConnectionStatus(bool value);

    bool getWiFiConnectionStatus();
    void setWiFiConnectionStatus(bool value);
};

#endif