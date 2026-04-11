#include <Arduino.h>
#include <WiFi.h>

#include "../env.h"
#include "robot/robot.h"
#include "robot/pidController.h"
#include "tests.h"
#include "utils/config.h"
#include "utils/logging.h"
#include "wifi/connection.h"

uint32_t frame = 0;
uint32_t previous_time = 0;

WiFiClient client;
Robot robot;

// Setup gets run at startup
void setup() {
    delay(5000);
    // client.connect(SERVER_IP, SERVER_PORT);

    if (LOGGING_LEVEL > 0) Serial.begin(115200);

    previous_time = micros();

    // sleepy_test(robot);
    robot.center();
}

// After setup gets run, loop is run over and over as fast ass possible
void loop() {
    uint32_t delta = micros() - previous_time;
    previous_time = micros();

    robot.tick(frame, delta);
    // square_test(robot);

    frame++;
}