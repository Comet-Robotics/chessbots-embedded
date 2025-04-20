#include "robot/trapezoidalProfile.h"
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace std;

double updateTrapezoidalProfile(MotionProfile &profile, double dt) {
    // distanceToGo = positive, you're still behind the target. || distanceToGo = negative, you've overshot.
    double distanceToGo = profile.targetPosition - profile.currentPosition;

    //Calculate the amount of acceleration we need to slow down smoothly
    double changeInDeceleration = (2 * (1 - (abs(distanceToGo) / abs(profile.targetPosition)))) * profile.maxAcceleration; 
    double changeInAcceleration = 1.4 * (distanceToGo / profile.targetPosition) * profile.maxAcceleration;

    
    // Acceleration phase
    if (abs(profile.currentVelocity) < profile.maxVelocity && abs(distanceToGo) > 0)
        profile.currentVelocity += changeInAcceleration * dt * (distanceToGo > 0 ? 1 : -1);

    // Deceleration if we’re getting close to target
    // Formula is v^2 / 2a
    double stoppingDistance = .5 * profile.targetPosition;//(profile.currentVelocity * profile.currentVelocity) / (2.0 * profile.maxAcceleration);
    
    if (abs(distanceToGo) <= stoppingDistance)
        profile.currentVelocity -= changeInDeceleration * dt * (profile.currentVelocity > 0 ? 1 : -1);

    // Determine acceleration or deceleration
    if (abs(profile.currentVelocity) > profile.maxVelocity)
        profile.currentVelocity = profile.maxVelocity * (profile.currentVelocity > 0 ? 1 : -1);


    // if (abs(distanceToGo) < 500)
    //     profile.currentVelocity = 0;

    return profile.currentVelocity;
}
