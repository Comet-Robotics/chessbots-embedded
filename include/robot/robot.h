#pragma once

#include <Arduino.h>
#include <optional>

#include "robot/lights.h"
#include "robot/motion-controller.h"
#include "robot/motor.h"
#include "robot/pid.h"
#include "utils/config.h"
#include "utils/geometry.h"

enum DriveType
{
    STOPPED,
    MANUAL,
    MOTION_CONTROL,
};

class Robot {
    public:
        Robot();

        static int batteryLevel();
        MotionController::MotionPhase motion_status();

        // Runs all the necessary processing for each tick of the global event loop
        void tick(uint32_t frame, uint32_t delay);
        
        void center(std::optional<std::string> id);
        void drive(double tiles, std::string id);
        void drive(Coordinate2D goal_pos, double goal_angle);
        void drive(std::tuple<double, double>& powers);

        void turn(double angleRadians, std::string id);
        
        void start();
        void stop();

        friend class MotionController;

    private:
        // Components
        Motor left;
        Motor right;

        Light front_left_light;
        Light front_right_light;
        Light back_left_light;
        Light back_right_light;

        // State
        double rotation;
        Coordinate2D position;
        MotionController motion_controller;
        
        DriveType drive_mode;
        CenteringStatus centeringStatus;
        std::optional<std::string> centeringID;

        void center_tick(uint32_t delay);
        void pid_tick(uint32_t delay);

        void print_status(uint32_t delay);
};

extern Robot robot;