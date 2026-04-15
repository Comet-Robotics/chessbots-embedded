#include <Arduino.h>
#include <WiFi.h>

#include "../env.h"
#include "robot/robot.h"
#include "robot/pidController.h"
#include "tests.h"
#include "utils/config.h"
#include "utils/logging.h"
#include "wifi/connection.h"
#include "wifi/packet.h"

uint32_t frame = 0;
uint32_t previous_time = 0;

Robot robot;

void setup() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    if (LOGGING_LEVEL > 0) {
        Serial.begin(115200);
    };

    // sleepy_test(robot);
}

void loop() {
    delay(10); // We want to run at ~100 fps to standardize motor power <-> speed
    uint32_t delta = micros() - previous_time;
    previous_time = micros();

    connection_check_reconnect();
    auto packet = recv_packet();
    if (packet.has_value()) {
        handle_packet(robot, packet.value());
    }
    
    robot.tick(frame, delta);

    // center_test(robot);
    // line_test(robot);
    // square_test(robot);
    // circle_test(robot);

    frame++;
}