// Associated Header File
#include "robot/control/robot.h"

// Built-In Libraries
#include "Arduino.h"
#include <queue>
#include <tuple>

// Custom Libraries
#include "utils/logging.h"
#include "utils/timer.h"
#include "utils/config.h"
#include "utils/logging.h"
#include "utils/geometry.h"
#include "utils/status.h"
#include "robot/control/motor.h"
#include "robot/control/lights.h"
#include "wifi/connection.h"
#include "robot/pidController.h"
#include "robot/driveTest.h"
#include "../../../env.h"
#include <algorithm>
#include "utils/functions.h"

MotionController::MotionController()
    :   DistVelocityController(0.5, 0.01, 0.15, -1, +1, 0.0),
        AVelocityController(0.5, 0.1, 0.15, -1, +1, 0.0)
{}

void MotionController::set_goal(Coordinate2D _goal_destination, double _goal_angle) {
    goal_angle = _goal_angle;
    goal_position = _goal_destination;
}

std::tuple<double, double> MotionController::update_speeds(Coordinate2D position, double angle, double dt) {
    double dist_err = position.distance_to(goal_position);

    // So that the robot will not back up from a point in front of it infinitely
    bool is_behind = position.is_behind(angle, goal_position);
    if (!is_behind) {
        dist_err = -dist_err;
    }
    
    if (abs(dist_err) < 1) {
        if (phase == TRAVELLING) {
            DistVelocityController.Reset();
            AVelocityController.Reset();
        }

        phase = ALIGNING;

        double angular_vel = AVelocityController.Compute(goal_angle, angle, dt);

       return std::make_tuple(-angular_vel, angular_vel);
    } else {
        if (phase == ALIGNING) {
            DistVelocityController.Reset();
            AVelocityController.Reset();
        }

        phase = TRAVELLING;

        double temp_goal_angle = position.angle_to(goal_position);

        double vel = DistVelocityController.Compute(0, dist_err, dt);
        double angular_vel = AVelocityController.Compute(temp_goal_angle, angle, dt);

        // https://aleksandarhaber.com/tutorial-on-simple-position-controller-for-differential-drive-robot-with-simulation-and-animation-in-python/
        double left_speed = (vel / WHEEL_RADIUS_CM) - ((TRACK_WIDTH_CM * angular_vel) / (2 * WHEEL_RADIUS_CM));
        double right_speed = (vel / WHEEL_RADIUS_CM) + ((TRACK_WIDTH_CM * angular_vel) / (2 * WHEEL_RADIUS_CM)); 

        return std::make_tuple(left_speed, right_speed);
    };
}

void MotionController::print_status() {
    serial_printf(DebugLevel::NONE, "MotionController status: %d -- goal_position: (%f, %f), goal_angle: %f\n", phase, goal_position.x, goal_position.y, goal_angle);
}

void MotionController::reset() {
    DistVelocityController.Reset();
    AVelocityController.Reset();
}

Robot::Robot()
    :   left(false, MOTOR_A_PIN1, MOTOR_A_PIN2, ENCODER_A_PIN1, ENCODER_A_PIN2),
        right(true, MOTOR_B_PIN1, MOTOR_B_PIN2, ENCODER_B_PIN1, ENCODER_B_PIN2),

        front_left_light(PHOTODIODE_A_PIN),
        front_right_light(PHOTODIODE_B_PIN),
        back_left_light(PHOTODIODE_C_PIN),
        back_right_light(PHOTODIODE_D_PIN)
{}

void Robot::print_status() {
    serial_clear();
    activateIR();
    serial_printf(
        DebugLevel::INFO,
        "position: (%fcm, %fcm) rotation: %frad \n"
        "Centering status: %d\n\n"

        "left power: %f right power: %f \n"
        "left wheel: %fcm right wheel: %fcm\n"
        "left enc raw: %d right enc raw %d\n\n"

        "front lights -- left: %hd discrete %d right: %hd discrete %d\n"
        "back lights -- left: %hd discrete %d right: %hd discrete %d\n\n",

        position.x, position.y, rotation,
        centeringStatus,

        left.power(), right.power(),
        left.dist(), right.dist(),
        left.raw_dist(), right.raw_dist(),
        

        front_left_light.raw_value(), front_left_light.value(), front_right_light.raw_value(), front_right_light.value(),
        back_left_light.raw_value(), back_left_light.value(), back_right_light.raw_value(), back_right_light.value()
    );

    motion_controller.print_status();

}

int Robot::batteryLevel() {
    return analogRead(BATTERY_VOLTAGE_PIN) - BATTERY_VOLTAGE_OFFSET;
}

void Robot::tick(unsigned long frame, uint32_t delay) {
    // Pass through tick, update all sensors / motors
    left.tick();
    right.tick();

    front_left_light.tick();
    front_right_light.tick();
    back_left_light.tick();
    back_right_light.tick();

    // Calculate new position and rotation
    double distance_sum = right.tick_dist() + left.tick_dist();
    double distance_offset = right.tick_dist() - left.tick_dist();

    double d_angle = (distance_offset)/TRACK_WIDTH_CM;

    // != on a double is fine here, because the robot may not move at all.
    if(d_angle != 0){
        double temp = (TRACK_WIDTH_CM * distance_sum)/(2*(distance_offset));
        Coordinate2D delta(sin(rotation + d_angle) - sin(rotation), cos(rotation) - cos(rotation + d_angle));
        position = position.transform(delta.scale(temp));
    } else {
        double totalDist = TIRE_RADIUS*(distance_sum)/2;
        Coordinate2D delta(cos(rotation + d_angle), sin(rotation + d_angle));
        position = position.transform(delta.scale(totalDist));
    }

    rotation = rotation + d_angle;

    center_tick(delay);
    pid_tick(delay);

    
    if (frame % 64 == 0) {
        print_status();
    }
}

void Robot::pid_tick(uint32_t delay) {    
    std::tuple<double, double> motor_speeds = motion_controller.update_speeds(position, rotation, (double)delay/1000);

    drive(motor_speeds, "NULL");
}

void Robot::center_tick(uint32_t delay) {
    if (centeringStatus == NOT_CENTERING) {
        return;
    }

    if (centeringStatus == STARTED) {
        // Set the goal position to a unit vector ahead of the robot
        motion_controller.set_goal(position.transform(Coordinate2D(rotation)), 0);

        // The two front sensors crossed at the same time, skip aligning step
        if (front_left_light.held_value() && front_right_light.held_value()) {
            centeringStatus = ALIGNED_EDGE_1;
        }

        if (front_left_light.held_value() ^ front_right_light.held_value()) {
            centeringStatus = ALIGNING_EDGE_1;
        }
    }

    if (centeringStatus == ALIGNING_EDGE_1) {        
        if (front_left_light.held_value() && front_right_light.held_value()) {
            rotation = M_PI / 2;
            position.y = -3;
            centeringStatus = ALIGNED_EDGE_1;
        }

        // If the left one crossed latest
        if (front_left_light.last_changed_time() > front_right_light.last_changed_time()) {
            // Turn left
            motion_controller.set_goal(position, rotation + 1);
        } else {
            // Turn right
            motion_controller.set_goal(position, rotation - 1);
        }
    }

    if (centeringStatus == ALIGNED_EDGE_1) {
        motion_controller.set_goal(position.transform(Coordinate2D(rotation)), 0);
    }
}

void Robot::center() {
    if (centeringStatus == NOT_CENTERING) {
        centeringStatus = STARTED;
    }
}

void Robot::drive(Coordinate2D goal_pos, double goal_angle) {
    motion_controller.set_goal(goal_pos, goal_angle);
}

void Robot::drive(float tiles, std::string id) {
    const float TILE_SIZE_CM = 24 * 2.54;
    motion_controller.set_goal(Coordinate2D(rotation).scale(TILE_SIZE_CM), rotation);
}

// Drives the wheels according to the powers set. Negative is backwards, Positive forwards
void Robot::drive(std::tuple<double, double>& powers, std::string id) {
    left.power(std::get<0>(powers));
    right.power(std::get<1>(powers));

    //we only send null as id during our test drive. The only other time this drive method is called will be
    //when the server sends it, meaning it will have an id to send back.
    if (id != "NULL") { sendActionSuccess(id); }
}

//turns the given amount in radians, CCW
void Robot::turn(float angleRadians, std::string id) {
}

void Robot::stop() {
    // Set the goal to where we are right now, so the robot doesn't move
    motion_controller.set_goal(position, rotation);
    
    serialLogln("Bot Stopped!", 2);
}