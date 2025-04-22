#include "robot/trapezoidalProfile.h"
#include "utils/logging.h"
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace std;

double updateTrapezoidalProfile(MotionProfile &profile, double dt) {
    // distanceToGo = positive, you're still behind the target. || distanceToGo = negative, you've overshot.
    double distanceToGo = profile.targetPosition - profile.currentPosition;

    // Formula is v^2 / 2a
    double stoppingDistance = (profile.maxVelocity * profile.maxVelocity) / (2.0 * profile.maxAcceleration);

    double changeInVelocity = 0;
    
    // Acceleration phase
    if (abs(profile.currentVelocity) < profile.maxVelocity && abs(distanceToGo) > 0)
        changeInVelocity = profile.maxAcceleration * dt * (distanceToGo > 0 ? 1 : -1);

    // Deceleration if we’re getting close to target
    
    if (abs(distanceToGo) <= stoppingDistance)
        changeInVelocity = -1 * profile.maxAcceleration * dt * (profile.currentVelocity > 0 ? 1 : -1);

        profile.requiredVelocity += changeInVelocity;
    
    if (abs(profile.currentVelocity) > profile.maxVelocity)
        profile.currentVelocity = profile.maxVelocity * (profile.currentVelocity > 0 ? 1 : -1);

    // serialLog((char *)"Motion profile is outputting: ", 3);
    // serialLog(float(profile.currentVelocity), 3);
    // serialLogln((char *)",", 3);

    return profile.requiredVelocity;
}
