#include <Arduino.h>
#include <queue>
#include <tuple>
#include <math.h>
#include <algorithm>
#include <optional>

#include "robot/robot.h"

#include "../../env.h"
#include "robot/lights.h"
#include "robot/motor.h"
#include "robot/pid.h"
#include "utils/config.h"
#include "utils/functions.h"
#include "utils/geometry.h"
#include "utils/logging.h"
#include "utils/logging.h"
#include "wifi/connection.h"

Robot robot = Robot();

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
        "FPS: %lu (%luus) Voltage: %d\n"
        "WiFi status: %d Connected: %d\n"

        "Position: (%fcm, %fcm)\n"
        "Rotation: %frad (%fdeg)\n"
        "Drive mode: %d Centering status: %d\n"

        "\n"

        "Motors:\n"
        "  Left:\n"
        "    power: %f (%d duty)\n"
        "    distance: %fcm (%d raw)\n"
        "    saved: %fcm\n"
        "  Right:\n"
        "    power: %f (%d duty)\n"
        "    distance: %fcm (%d raw)\n"
        "    saved: %fcm\n"

        "\n"

        "Lights:\n"
        "  Front:\n"
        "    Left: %hd (disc %d), (held %d) (changed %lu)\n"
        "    Right: %hd (disc %d), (held %d) (changed %lu)\n"
        "  Back:\n"
        "    Left: %hd (disc %d), (held %d) (changed %lu)\n"
        "    Right: %hd (disc %d), (held %d) (changed %lu)\n\n",

        fps, delay, Robot::batteryLevel(),
        WiFi.status(), client.connected(),

        position.x, position.y, rotation, RAD_TO_DEG * rotation,
        drive_mode, centeringStatus,

        left.power(), left.duty(), left.dist(), left.raw_dist(), left.get_saved(),
        right.power(), right.duty(), right.dist(), right.raw_dist(), right.get_saved(),

        front_left_light.raw_value(), front_left_light.value(), front_left_light.held_value(), front_left_light.last_changed_time(),
        front_right_light.raw_value(), front_right_light.value(), front_right_light.held_value(), front_right_light.last_changed_time(),

        back_left_light.raw_value(), back_left_light.value(), back_left_light.held_value(), back_left_light.last_changed_time(),
        back_right_light.raw_value(), back_right_light.value(), back_right_light.held_value(), back_right_light.last_changed_time()
    );

    motion_controller.print_status();

}

int Robot::batteryLevel() {
    return analogRead(BATTERY_VOLTAGE_PIN) + BATTERY_VOLTAGE_OFFSET;
}

MotionController::MotionPhase Robot::motion_status() {
    return motion_controller.phase();
}

void Robot::tick(uint32_t frame, uint32_t delay) {
    // Pass through tick, update all sensors / motors
    left.tick(delay);
    right.tick(delay);

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
        drive(motor_speeds);
    } else if (drive_mode == DriveType::MOTION_CONTROL) {
        motion_controller.tick(delay);
    }
    
    if (frame % 32 == 0) {
        print_status(delay);
    }
}

#define CENTER_MOTOR_SPEED .35
#define LIGHT_DISTANCE 17 // In CM
#define BACKUP_DIST 20.0
void Robot::center_tick(uint32_t delay) {
    if (centeringStatus == NOT_CENTERING) {
        return;
    }

    if (centeringStatus == STARTED) {
        drive_mode = DriveType::MANUAL;

        auto motor_speeds = std::make_tuple(CENTER_MOTOR_SPEED, CENTER_MOTOR_SPEED);
        drive(motor_speeds);

        if (front_left_light.held_value() || front_right_light.held_value()) {
            left.save_dist();
            right.save_dist();

            centeringStatus = ALIGNING_EDGE_1;
        }
    }

    if (centeringStatus == ALIGNING_EDGE_1) {
        drive_mode = DriveType::MANUAL;

        auto motor_speeds = std::make_tuple(CENTER_MOTOR_SPEED, CENTER_MOTOR_SPEED);
        drive(motor_speeds);
        double delta_dist;

        if (front_left_light.held_value() && front_right_light.held_value()) {
            if (front_left_light.last_changed_time() > front_right_light.last_changed_time()) {
                delta_dist = left.dist() - left.save_dist();
                rotation = atan(delta_dist / LIGHT_DISTANCE);
            } else {
                delta_dist = right.dist() - right.save_dist();
                rotation = -atan(delta_dist / LIGHT_DISTANCE);
            }

            serial_printf(DebugLevel::INFO, "ROTATION: %fdeg, delta_dist: %f", rotation * RAD_TO_DEG, delta_dist);
            position.y = BACKUP_DIST;
            rotation = rotation + (M_PI /  2);

            centeringStatus = ALIGNED_EDGE_1;
        }
    }

    if (centeringStatus == ALIGNED_EDGE_1) {
        drive_mode = DriveType::MOTION_CONTROL;

        motion_controller.set_goal(Coordinate2D(position.x, 0.0), 0, std::nullopt);

        if (motion_controller.phase() == MotionController::MotionPhase::ARRIVED) {
            front_left_light.reset();
            front_right_light.reset();
            
            motion_controller.set_goal(Coordinate2D(position.x + 100.0, 0.0), 0, std::nullopt);
            centeringStatus = CENTERED_Y_AXIS;
        }
    }

    // Similar to the STARTING status
    if (centeringStatus == CENTERED_Y_AXIS) {
        drive_mode = DriveType::MOTION_CONTROL;

        if (front_left_light.held_value() && front_right_light.held_value()) {
            position.x = BACKUP_DIST;
 
            motion_controller.set_goal(Coordinate2D(0.0, 0.0), M_PI / 2, centeringID);
            centeringID = std::nullopt;
            centeringStatus = NOT_CENTERING;
        }
    }
}

void Robot::center(std::optional<std::string> id) {
    if (centeringStatus == NOT_CENTERING) {
        centeringStatus = STARTED;
        centeringID = id;

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
void Robot::drive(std::tuple<double, double>& powers) {
    left.set_target_speed(std::get<0>(powers));
    right.set_target_speed(std::get<1>(powers));
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