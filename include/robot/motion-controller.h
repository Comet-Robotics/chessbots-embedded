#pragma once

#include <optional>
#include <string>

#include "robot/pid.h"
#include "utils/geometry.h"

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
        void tick(uint32_t delta);
        
        void print_status();
        void reset();
    private:
        std::optional<std::string> actionID;

        PIDController DistVelocityController; 
        PIDController AVelocityController;

        MotionPhase _phase;
        MotionPhase _prev_phase;
        
        double goal_angle;
        Coordinate2D goal_position;
};