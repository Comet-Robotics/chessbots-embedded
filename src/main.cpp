#include <Arduino.h>
#include <WiFi.h>

#include "../env.h"
#include "robot/robot.h"
#include "robot/pid.h"
#include "tests.h"
#include "utils/config.h"
#include "utils/logging.h"
#include "wifi/connection.h"
#include "wifi/packet.h"

uint32_t frame = 0;
uint32_t previous_time = 0;

uint32_t connection_backoff = 1;
uint32_t connection_backoff_next_time = 0;

void setup() {
    #if ONLINE
        WiFi.mode(WIFI_STA);
        // reconnect_wifi();
        // client.connect(SERVER_IP, SERVER_PORT);
        // send_handshake();
    #endif

    if (LOGGING_LEVEL > 0) {
        Serial.begin(115200);
    };

    // sleepy_test(robot);
    // hardware_test(robot);
}

void loop() {
    // delay(5); // We want to run at ~100 fps to standardize motor power <-> speed
    uint32_t delta = micros() - previous_time;
    previous_time = micros();

    #if ONLINE
        if (!connected_wifi()) {
            reconnect_wifi();
        }

        if (!connected_server()) {
            reconnect_server();
        }

        auto packet = recv_packet();
        if (packet.has_value()) {
            handle_packet(robot, packet.value());
        }
    #endif

    robot.tick(frame, delta);

    // center_test(robot);
    // line_test(robot);
    // square_test(robot);
    // circle_test(robot);
    // small_angle_test(robot);
    // ir_test(robot);

    frame++;
}