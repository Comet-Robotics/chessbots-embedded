#include <Arduino.h>

#include "tests.h"

#include "robot/robot.h"
#include "utils/geometry.h"
#include "utils/logging.h"

// Turns off the motion controller on the robot
void sleepy_test(Robot& r) {
    r.stop();
}

void center_test(Robot& r) {
    unsigned long time_seconds = millis() / 1000;

    if (time_seconds % 30 == 10) {
        r.center(std::nullopt);
    }
}

// Test the distance PID controller
void line_test(Robot& r) {
    unsigned long time_seconds = millis() / 1000;

    Coordinate2D goal;
    double rotation = 0;

    if (time_seconds > 5) {
        goal = Coordinate2D(100, 0);
    }

    if (time_seconds > 15) {
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

int goal_idx = 0;

// Test all of MotionController
void square_test(Robot& r) {
    unsigned long time_seconds = millis() / 1000;
    Coordinate2D goal;
    double rotation;

    if (goal_idx == 0) {
        goal = Coordinate2D(0, 100);
        rotation = M_PI / 2;
    }

    if (goal_idx == 1) {
        goal = Coordinate2D(0, 0);
        rotation = 0;
    }

    if (goal_idx == 2) {
        goal = Coordinate2D(100, 0);
    }
    
    if (goal_idx == 3) {
        goal = Coordinate2D(100, 100);
    }

    if (goal_idx == 4) {
        goal = Coordinate2D(0, 100);
    }

    if (goal_idx > 5) {
        goal = Coordinate2D(0, 0);
    }


    if (r.motion_status() == MotionController::MotionPhase::ARRIVED) {
        r.drive(goal, rotation);
        goal_idx += 1;
    }
}