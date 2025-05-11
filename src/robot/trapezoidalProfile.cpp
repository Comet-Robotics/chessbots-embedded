#include "robot/trapezoidalProfile.h"
#include "utils/logging.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <cstdint>

using namespace std;

double updateTrapezoidalProfile(MotionProfile &profile, double dt, int8_t framesUntilprint) {
    // distanceToGo = positive, you're still behind the target. || distanceToGo = negative, you're ahead.
    double distanceToGo = profile.targetPosition - profile.currentPosition;
    
    double changeInVelocity = 0, stoppingDistance;

    //kickstart and begin velocity
    if (profile.currentVelocity == 0) {
        profile.targetVelocity = 0.15 * profile.maxVelocity * (distanceToGo > 0 ? 1 : -1);
    }

    // Formula is v^2 / 2a
    stoppingDistance = (profile.currentVelocity * profile.currentVelocity) / (2.0 * profile.maxAcceleration);
    double decrement_ratio = -(1 - (profile.currentPosition/profile.targetPosition));
    // if (profile.targetPosition != 0){
    //     stoppingDistance  = profile.currentPosition + (decrement_ratio*profile.currentVelocity*dt);
    // }
    // else{
    //     stoppingDistance = (profile.currentVelocity * profile.currentVelocity) / (2.0 * profile.maxAcceleration);
    // }
    
    // Acceleration phase
    if (fabs(profile.currentVelocity) < profile.maxVelocity && fabs(distanceToGo) > 0)
        changeInVelocity = profile.maxAcceleration * dt * (distanceToGo > 0 ? 1 : -1);

    // Deceleration if we’re getting close to target
    if (fabs(distanceToGo) <= stoppingDistance && profile.targetPosition > 0){
        //making it dependent on leftover distance/dt instead of maxAcceleration*dt
        changeInVelocity = -(profile.currentPosition/profile.targetPosition) * profile.maxAcceleration * dt * (profile.currentVelocity > 0 ? 1 : -1);
    }

    // Deceleration if we’re getting close to target
    if (fabs(distanceToGo) <= stoppingDistance){
        //making it dependent on leftover distance/dt instead of maxAcceleration*dt
        changeInVelocity = -1 * profile.maxAcceleration * dt * (profile.currentVelocity > 0 ? 1 : -1);
    }

    profile.targetVelocity += changeInVelocity;

    //clip off velocity if it's too high
    if (fabs(profile.targetVelocity) > profile.maxVelocity)
        profile.targetVelocity = profile.maxVelocity * (profile.targetVelocity > 0 ? 1 : -1);

    if(fabs(distanceToGo) < 50)
        profile.targetVelocity = 0;

    if(framesUntilprint == 0)
    {
        serialLog("Motion profile is outputting: ", 3);
        serialLog(float(profile.targetVelocity), 3);
        serialLog(",", 3);

        serialLog("Target Position: ", 3);
        serialLog(float(profile.targetPosition), 3);
        serialLog(",", 3);

        serialLog("Stopping position: ", 3);
        serialLog(float(stoppingDistance), 3);
        serialLogln(",", 3);
    }

    return profile.targetVelocity;
}
