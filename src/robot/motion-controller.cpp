#include <Arduino.h>

#include <tuple>
#include "robot/robot.h"
#include "wifi/connection.h"

#include "utils/config.h"
#include "utils/geometry.h"
#include "utils/logging.h"

#include "robot/motion-controller.h"
 
MotionController::MotionController()
    :   DistVelocityController(0.1, 0.2, 0.1, -3, +3, 0.0),
        AVelocityController(.1, 0.4, 0.1, -1, +1, 0.0)
{}

MotionController::MotionPhase MotionController::phase() {
    double dist_err = robot.position.distance_to(goal_position);
    double angle_err = robot.rotation - goal_angle;

    if (abs(dist_err) < 8 && abs(angle_err) < .01) {
        digitalWrite(ONBOARD_LED_PIN, HIGH);

        return ARRIVED;
    } else if (abs(dist_err) < 3) {
        digitalWrite(ONBOARD_LED_PIN, LOW);

        return ALIGNING;
    } else {
        digitalWrite(ONBOARD_LED_PIN, LOW);

        return TRAVELLING;
    }
}

void MotionController::set_goal(Coordinate2D _goal_destination, double _goal_angle, std::optional<std::string> id) {
    actionID = id;
    goal_angle = _goal_angle;
    goal_position = _goal_destination;
}

void MotionController::tick(uint32_t delta) {
    double dist_err = robot.position.distance_to(goal_position);

    _prev_phase = _phase;
    _phase = phase();

    if (_phase == ARRIVED) {
        if (actionID.has_value()) {
            send_success(actionID.value());
            actionID = std::nullopt;
        }

        auto powers = std::make_tuple(0.0, 0.0);
        robot.drive(powers);
    } else if (_phase == ALIGNING) {
        double angular_vel = AVelocityController.Compute(goal_angle, robot.rotation, (double) delta / 1000000);
       auto powers = std::make_tuple(-angular_vel, angular_vel);
       robot.drive(powers);
    } else {
        if (_prev_phase != TRAVELLING) {
            DistVelocityController.Reset();
            AVelocityController.Reset();
        }

        double temp_goal_angle;
        if (robot.position.is_behind(robot.rotation, goal_position)) {
            temp_goal_angle = goal_position.angle_to(robot.position);
        } else {
            dist_err = -dist_err;
            temp_goal_angle = robot.position.angle_to(goal_position);
        }

        double vel = DistVelocityController.Compute(0, dist_err, (double) delta / 1000000);
        double angular_vel = AVelocityController.Compute(temp_goal_angle, robot.rotation, (double) delta / 1000000);

        // https://aleksandarhaber.com/tutorial-on-simple-position-controller-for-differential-drive-robot-with-simulation-and-animation-in-python/
        auto powers = std::make_tuple(
            (vel / WHEEL_RADIUS_CM) - ((TRACK_WIDTH_CM * angular_vel) / (2 * WHEEL_RADIUS_CM)),
            (vel / WHEEL_RADIUS_CM) + ((TRACK_WIDTH_CM * angular_vel) / (2 * WHEEL_RADIUS_CM))
        ); 

        robot.drive(powers);
    };
}

void MotionController::print_status() {
    serial_printf(DebugLevel::DEBUG, "MotionController status: %d\n  goal_angle: %f (%fdeg)\n  goal_position: (%f, %f)", _phase, goal_angle, RAD_TO_DEG *goal_angle, goal_position.x, goal_position.y);
}

void MotionController::reset() {
    DistVelocityController.Reset();
    AVelocityController.Reset();
}
