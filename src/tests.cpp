#include <Arduino.h>

#include "tests.h"

#include "robot/control/robot.h"
#include "utils/geometry.h"

// Test the distance PID controller
void line_test(Robot& r) {
    unsigned long time_seconds = millis() / 1000;

    Coordinate2D goal;
    double rotation = 0;

    if (time_seconds > 5) {
        goal = Coordinate2D(100, 0);
    }

    if (time_seconds > 10) {
        goal = Coordinate2D(100, 100);
    }

    if (time_seconds > 15) {
        goal = Coordinate2D(0, 100);
    }

    if (time_seconds > 20) {
        goal = Coordinate2D(0, 0);
    }

    r.drive(goal, rotation);
}

// Test the angular PID controller
void circle_test(Robot& r) {
    unsigned long time_seconds = millis() / 1000;

    Coordinate2D goal(00, 0.0);
    double rotation = 0;

    if (time_seconds > 5) {
        rotation = 2 * M_PI;
    }

    if (time_seconds > 10) {
        rotation =  0;
    }

    if (time_seconds > 15) {
        rotation = 4 * M_PI;
    }

    if (time_seconds > 20) {
        rotation = 0;
    }

    r.drive(goal, rotation);
}

// Test all of MotionController
void square_test(Robot& r) {
    unsigned long time_seconds = millis() / 1000;

    Coordinate2D goal;
    double rotation = 0;

    if (time_seconds > 5) {
        goal = Coordinate2D(100, 0);
    }

    if (time_seconds > 10) {
        goal = Coordinate2D(100, 100);
    }

    if (time_seconds > 15) {
        goal = Coordinate2D(0, 100);
    }

    if (time_seconds > 20) {
        goal = Coordinate2D(0, 0);
    }

    r.drive(goal, rotation);
}