#include <Arduino.h>
#include <queue>
#include <tuple>
#include <algorithm>
#include <optional>

#include "robot/robot.h"

#include "../../env.h"
#include "robot/lights.h"
#include "robot/motor.h"
#include "robot/pidController.h"
#include "utils/config.h"
#include "utils/functions.h"
#include "utils/geometry.h"
#include "utils/logging.h"
#include "utils/logging.h"
#include "wifi/connection.h"

MotionController::MotionController()
    :   DistVelocityController(0.1, 0.2, 0.1, -1.5, +1.5, 0.0),
        AVelocityController(.1, 0.4, 0.1, -.4, +.4, 0.0)
{}

MotionController::MotionPhase MotionController::phase() {
    return _phase;
}

void MotionController::set_goal(Coordinate2D _goal_destination, double _goal_angle, std::optional<std::string> id) {
    actionID = id;
    goal_angle = _goal_angle;
    goal_position = _goal_destination;
}

std::tuple<double, double> MotionController::update_speeds(Coordinate2D position, double angle, double dt) {
    double dist_err = position.distance_to(goal_position);
    double angle_err = angle - goal_angle;
    
    if (abs(dist_err) < 4 && abs(angle_err) < .01) {
        _phase = ARRIVED;

        digitalWrite(ONBOARD_LED_PIN, HIGH);

        if (actionID.has_value()) {
            send_success(actionID.value());
            actionID = std::nullopt;
        }

        // C++ let me compile without this return statement
        // While on -Wall and -Wextra.
        return std::make_tuple(0.0, 0.0);
    } else if (abs(dist_err) < 2) {
        if (_phase != ALIGNING) {
            digitalWrite(ONBOARD_LED_PIN, LOW);
        }

        _phase = ALIGNING;

        double angular_vel = AVelocityController.Compute(goal_angle, angle, dt);

       return std::make_tuple(-angular_vel, angular_vel);
    } else {
        if (_phase != TRAVELLING) {
            DistVelocityController.Reset();
            AVelocityController.Reset();

            digitalWrite(ONBOARD_LED_PIN, LOW);
        }

        bool goal_infront = position.is_behind(angle, goal_position);

        double temp_goal_angle;

        if (!goal_infront) {
            dist_err = -dist_err;
            temp_goal_angle = position.angle_to(goal_position);
        } else {
            temp_goal_angle = goal_position.angle_to(position);
        }

        _phase = TRAVELLING;

        double vel = DistVelocityController.Compute(0, dist_err, dt);
        double angular_vel = AVelocityController.Compute(temp_goal_angle, angle, dt);

        // https://aleksandarhaber.com/tutorial-on-simple-position-controller-for-differential-drive-robot-with-simulation-and-animation-in-python/
        double left_speed = (vel / WHEEL_RADIUS_CM) - ((TRACK_WIDTH_CM * angular_vel) / (2 * WHEEL_RADIUS_CM));
        double right_speed = (vel / WHEEL_RADIUS_CM) + ((TRACK_WIDTH_CM * angular_vel) / (2 * WHEEL_RADIUS_CM)); 

        return std::make_tuple(left_speed, right_speed);
    };
}

void MotionController::print_status() {
    serial_printf(DebugLevel::DEBUG, "MotionController status: %d\n  goal_angle: %f\n  goal_position: (%f, %f)", _phase, goal_angle, goal_position.x, goal_position.y);
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
        back_right_light(PHOTODIODE_D_PIN),
        drive_mode(MOTION_CONTROL)


{

    // Enable ESP and IR blasters
    pinMode(RELAY_IR_LED_PIN, OUTPUT);
    pinMode(ONBOARD_LED_PIN, OUTPUT);
}

void Robot::print_status(uint32_t delay) {
    activateIR();

    uint32_t fps = delay == 0 ? 0 : 1000000 / delay;
    serial_printf(
        DebugLevel::DEBUG,
        
        SERIAL_CLEAR
        "FPS: %lu (%luus) WiFi status: %d -- connected: %d\n"

        "Position: (%fcm, %fcm) rotation: %frad \n"
        "Drive mode: %d Centering status: %d\n"

        "\n"

        "Motors:\n"
        "  Left:\n"
        "    power: %f (%d duty)\n"
        "    distance: %fcm (%d raw)\n"
        "  Right:\n"
        "    power: %f (%d duty)\n"
        "    distance: %fcm (%d raw)\n"

        "\n"

        "Lights:\n"
        "  Front:\n"
        "    Left: %hd (disc %d), (held %d) (changed %lu)\n"
        "    Right: %hd (disc %d), (held %d) (changed %lu)\n"
        "  Back:\n"
        "    Left: %hd (disc %d), (held %d) (changed %lu)\n"
        "    Right: %hd (disc %d), (held %d) (changed %lu)\n\n",

        fps, delay, WiFi.status(), client.connected(),

        position.x, position.y, rotation,
        drive_mode, centeringStatus,

        left.power(), left.duty(), left.dist(), left.raw_dist(),
        right.power(), right.duty(), right.dist(), right.raw_dist(),

        front_left_light.raw_value(), front_left_light.value(), front_left_light.held_value(), front_left_light.last_changed_time(),
        front_right_light.raw_value(), front_right_light.value(), front_right_light.held_value(), front_right_light.last_changed_time(),

        back_left_light.raw_value(), back_left_light.value(), back_left_light.held_value(), back_left_light.last_changed_time(),
        back_right_light.raw_value(), back_right_light.value(), back_right_light.held_value(), back_right_light.last_changed_time()
    );

    motion_controller.print_status();

}

int Robot::batteryLevel() {
    return analogRead(BATTERY_VOLTAGE_PIN) - BATTERY_VOLTAGE_OFFSET;
}
MotionController::MotionPhase Robot::motion_status() {
    return motion_controller.phase();
}

void Robot::tick(uint32_t frame, uint32_t delay) {
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
        double totalDist = distance_sum / 2;
        Coordinate2D delta(cos(rotation + d_angle), sin(rotation + d_angle));
        position = position.transform(delta.scale(totalDist));
    }

    rotation = rotation + d_angle;

    center_tick(delay);

    if (drive_mode == DriveType::MANUAL) {
        // Don't do anything, motors are being manually controlled somewhere else
    } else if (drive_mode == DriveType::STOPPED) {
        auto motor_speeds = std::make_tuple(0.0, 0.0);
        drive(motor_speeds, "NULL");
    } else if (drive_mode == DriveType::MOTION_CONTROL) {
        auto motor_speeds = motion_controller.update_speeds(position, rotation, (double)delay / 1000000);
        drive(motor_speeds, "NULL");
    }
    
    if (frame % 64 == 0) {
        print_status(delay);
    }
}

#define CENTER_MOTOR_SPEED .18
void Robot::center_tick(uint32_t delay) {
    if (centeringStatus == NOT_CENTERING) {
        return;
    }

    if (centeringStatus == STARTED) {
        drive_mode = DriveType::MANUAL;

        auto motor_speeds = std::make_tuple(CENTER_MOTOR_SPEED, CENTER_MOTOR_SPEED);
        drive(motor_speeds, "NULL");

        if (front_left_light.held_value() || front_right_light.held_value()) {
            centeringStatus = ALIGNING_EDGE_1;
        }
    }

    if (centeringStatus == ALIGNING_EDGE_1) {
        drive_mode = DriveType::MANUAL;

        if (front_left_light.held_value() && front_right_light.held_value()) {
            rotation = M_PI / 2;
            position = Coordinate2D(position.x, 15.0);
            centeringStatus = ALIGNED_EDGE_1;
        }

        // If the left one crossed latest, that's the first one that hit the line
        if (front_left_light.last_changed_time() > front_right_light.last_changed_time()) {
            // Turn left
            auto motor_speeds = std::make_tuple(0.0, CENTER_MOTOR_SPEED);
            drive(motor_speeds, "NULL");
        } else {
            // Turn right
            auto motor_speeds = std::make_tuple(CENTER_MOTOR_SPEED, 0.0);
            drive(motor_speeds, "NULL");
        }
    }

    if (centeringStatus == ALIGNED_EDGE_1) {
        drive_mode = DriveType::MOTION_CONTROL;

        motion_controller.set_goal(Coordinate2D(position.x, 0.0), 0, std::nullopt);

        if (motion_controller.phase() == MotionController::MotionPhase::ARRIVED) {
            front_left_light.reset();
            front_right_light.reset();
            
            centeringStatus = CENTERED_Y_AXIS;
        }
    }

    // Similar to the STARTING status
    if (centeringStatus == CENTERED_Y_AXIS) {
        drive_mode = DriveType::MANUAL;

        auto motor_speeds = std::make_tuple(CENTER_MOTOR_SPEED, CENTER_MOTOR_SPEED);
        drive(motor_speeds, "NULL");

        if (front_left_light.held_value() && front_right_light.held_value()) {
            drive_mode = DriveType::MOTION_CONTROL;

            position.x = 15.0;
 
            motion_controller.set_goal(Coordinate2D(0.0, 0.0), M_PI / 2, std::nullopt);
            centeringStatus = NOT_CENTERING;
        }
    }
}

void Robot::center() {
    if (centeringStatus == NOT_CENTERING) {
        centeringStatus = STARTED;
        drive_mode = DriveType::MANUAL;

        front_left_light.reset();
        front_right_light.reset();
        back_left_light.reset();
        back_right_light.reset();
    }
}

void Robot::drive(Coordinate2D goal_pos, double goal_angle) {
    motion_controller.set_goal(goal_pos, goal_angle, std::nullopt);
}

void Robot::drive(double tiles, std::string id) {
    const float TILE_SIZE_CM = 24 * 2.54;
    
    Coordinate2D offset(rotation);
    offset = offset.scale(TILE_SIZE_CM * tiles);
    Coordinate2D destination = position.transform(offset);

    motion_controller.set_goal(destination, rotation, id);
}

// Drives the wheels according to the powers set. Negative is backwards, Positive forwards
void Robot::drive(std::tuple<double, double>& powers, std::string id) {
    left.power(std::get<0>(powers));
    right.power(std::get<1>(powers));
}

//turns the given amount in radians, CCW
void Robot::turn(double delta, std::string id) {
    motion_controller.set_goal(position, rotation + delta, id);
}

void Robot::start() {
    drive_mode = DriveType::MOTION_CONTROL;
    
    serial_printf(DebugLevel::DEBUG, "Bot Started!\n");
}

void Robot::stop() {
    drive_mode = DriveType::STOPPED;
    
    serial_printf(DebugLevel::DEBUG, "Bot Stopped!\n");
}