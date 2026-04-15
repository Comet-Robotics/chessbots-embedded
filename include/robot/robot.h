#pragma once

#include <Arduino.h>
#include <optional>

#include "robot/lights.h"
#include "robot/motor.h"
#include "robot/pidController.h"
#include "utils/config.h"
#include "utils/geometry.h"

enum DriveType
{
    STOPPED,
    MANUAL,
    MOTION_CONTROL,
};

enum CenteringStatus {
    NOT_CENTERING,
    
    // The robot has started centering
    STARTED,

    // Has hit the first edge, and is now aligning to it
    ALIGNING_EDGE_1,

    ALIGNED_EDGE_1,

    CENTERED_Y_AXIS,

    ALIGNED_EDGE_2,
};

class MotionController {
    public:
        enum MotionPhase {
            // Traveling to the destination
            TRAVELLING,

            // Aligning to the correct rotation
            ALIGNING,
            ARRIVED
        };

        MotionController();

        MotionPhase phase();
        void set_goal(Coordinate2D goal_destination, double goal_angle, std::optional<std::string> id);
        std::tuple<double, double> update_speeds(Coordinate2D position, double angle, double dt);
        
        void print_status();
        void reset();
    private:
        std::optional<std::string> actionID;

        PIDController DistVelocityController; 
        PIDController AVelocityController;

        MotionPhase _phase;
        
        double goal_angle;
        Coordinate2D goal_position;
};

class Robot {
    public:
        Robot();

        static int batteryLevel();
        MotionController::MotionPhase motion_status();

        // Runs all the necessary processing for each tick of the global event loop
        void tick(uint32_t frame, uint32_t delay);
        
        void center();
        void drive(double tiles, std::string id);
        void drive(Coordinate2D goal_pos, double goal_angle);
        void drive(std::tuple<double, double>& powers, std::string id);

        void turn(double angleRadians, std::string id);
        
        void start();
        void stop();
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
        
        void center_tick(uint32_t delay);
        void pid_tick(uint32_t delay);

        void print_status(uint32_t delay);
};