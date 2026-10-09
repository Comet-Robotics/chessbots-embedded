#include <Arduino.h>

#include <tuple>
#include "robot/robot.h"
#include "wifi/connection.h"

#include "utils/config.h"
#include "utils/geometry.h"
#include "utils/logging.h"

#include "robot/motion-controller.h"
 
MotionController::MotionController()
    :   DistVelocityController(0.8, 0.5, 0.1, -1.5, +1.5, 0.0),
        AVelocityController(.8, 0.5, 0.1, -.4, +.4, 0.0)
        // Maybe try this too? AVelocityController(.4, 0.4, 0.2, -.4, +.4, 0.0)
{}

MotionController::MotionPhase MotionController::phase() {
    double dist_err = robot.position.distance_to(_goal_position);
    double angle_err = robot.rotation - _goal_angle;

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

void MotionController::set_goal(Coordinate2D __goal_destination, double _goal_angle, std::optional<std::string> id) {
    actionID = id;
    _goal_angle = _goal_angle;
    _goal_position = __goal_destination;
}

Coordinate2D MotionController::goal_position() {
    return _goal_position;
}

double MotionController::goal_angle() {
    return _goal_angle;
}

void MotionController::tick(uint32_t delta) {
    double dist_err = robot.position.distance_to(_goal_position);

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
        double angular_vel = AVelocityController.Compute(_goal_angle, robot.rotation, (double) delta / 1000000);
       auto powers = std::make_tuple(-angular_vel, angular_vel);
       robot.drive(powers);
    } else {
        if (_prev_phase != TRAVELLING) {
            DistVelocityController.Reset();
            AVelocityController.Reset();
        }

        double temp_goal_angle;
        if (robot.position.is_behind(robot.rotation, _goal_position)) {
            temp_goal_angle = _goal_position.angle_to(robot.position);
        } else {
            dist_err = -dist_err;
            temp_goal_angle = robot.position.angle_to(_goal_position);
        }

        double vel = DistVelocityController.Compute(0, dist_err, (double) delta / 1000000);

        // There might still be a subtle angle problem here but hopefully that is fixed in alignment
        // Negative angle delta because we want PID to push angle in the positive direction
        double angular_vel = AVelocityController.Compute(0, -angle_delta(robot.rotation, temp_goal_angle), (double) delta / 1000000);

        // https://aleksandarhaber.com/tutorial-on-simple-position-controller-for-differential-drive-robot-with-simulation-and-animation-in-python/
        auto powers = std::make_tuple(
            (vel / WHEEL_RADIUS_CM) - ((TRACK_WIDTH_CM * angular_vel) / (2 * WHEEL_RADIUS_CM)),
            (vel / WHEEL_RADIUS_CM) + ((TRACK_WIDTH_CM * angular_vel) / (2 * WHEEL_RADIUS_CM))
        ); 

        robot.drive(powers);
    };
}

void MotionController::print_status() {
    serial_printf(DebugLevel::TRACE, "MotionController status: %d\n  goal_angle: %f (%fdeg)\n  goal_position: (%f, %f)", _phase, _goal_angle, RAD_TO_DEG * _goal_angle, _goal_position.x, _goal_position.y);
}

void MotionController::reset() {
    DistVelocityController.Reset();
    AVelocityController.Reset();
}
