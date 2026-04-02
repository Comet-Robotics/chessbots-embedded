#ifndef CHESSBOT_CONTROL_H
#define CHESSBOT_CONTROL_H

#include "Arduino.h"
#include "utils/config.h"
#include "utils/geometry.h"
#include "robot/trapezoidalProfile.h"
#include "robot/control/lights.h"
#include "robot/control/motor.h"
#include "robot/pidController.h"

enum OperatingMode
{
    POSITION,
    VELOCITY
};

enum DriveStatus {
    ONGOING,

    // The robot has reached its destination and is now reversing
    REACHED_REVERSING,

    // The robot has reached its destination, and reversing is not necessary
    REACHED
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
            ALIGNING
        };

        MotionController();

        void set_goal(Coordinate2D goal_destination, double goal_angle);
        std::tuple<double, double> update_speeds(Coordinate2D position, double angle, double dt);
        void print_status();
        void reset();
    private:
        PIDController DistVelocityController; 
        PIDController AVelocityController;

        MotionPhase phase;
        
        double goal_angle;
        Coordinate2D goal_position;
};

class Robot {
    public:
        Robot();

        static int batteryLevel();

        // Runs all the necessary processing for each tick of the global event loop
        void tick(unsigned long frame, uint32_t delay);
        
        void center();
        void drive(float tiles, std::string id);
        void drive(Coordinate2D goal_pos, double goal_angle);
        void drive(std::tuple<double, double>& powers, std::string id);
        void driveTicks(int tickDistance, std::string id);
        enum DriveStatus driveUntilNewTile();

        void turn(float angleRadians, std::string id);
        
        void stop();

        void test();
    
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
        
        CenteringStatus centeringStatus;
        
        void center_tick(uint32_t delay);
        void pid_tick(uint32_t delay);

        void print_status();
};

#endif