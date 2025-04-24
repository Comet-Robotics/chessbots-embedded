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
    //double stoppingDistance = 0.5 * (profile.maxVelocity * profile.maxVelocity) / (2.0 * profile.maxAcceleration);
    double stoppingDistance;
    double changeInVelocity;
    if (profile.targetPosition != 0){
        double decrement_ratio = -(1 - (profile.currentPosition/profile.targetPosition));
        stoppingDistance  = profile.currentPosition + (decrement_ratio*profile.currentVelocity*dt);
        changeInVelocity = 0;
    }
    else{
        stoppingDistance = 60000;
        changeInVelocity = 0;
    }
    
    // Acceleration phase
    if (abs(profile.currentVelocity) < profile.maxVelocity && abs(distanceToGo) > 0)
        changeInVelocity = profile.maxAcceleration * dt * (distanceToGo > 0 ? 1 : -1);

    // Deceleration if we’re getting close to target
    if (abs(distanceToGo) <= stoppingDistance && profile.targetPosition > 0){
        //making it dependent on leftover distance/dt instead of maxAcceleration*dt
        changeInVelocity = -(profile.currentPosition/profile.targetPosition) * profile.maxAcceleration * dt * (profile.currentVelocity > 0 ? 1 : -1);
    }

    // Deceleration if we’re getting close to target
    if (abs(distanceToGo) <= stoppingDistance){
        //making it dependent on leftover distance/dt instead of maxAcceleration*dt
        changeInVelocity = -1 * profile.maxAcceleration * dt * (profile.currentVelocity > 0 ? 1 : -1);
    }

    profile.targetVelocity += changeInVelocity;

    if (abs(profile.currentVelocity) > profile.maxVelocity)
        profile.currentVelocity = profile.maxVelocity * (profile.currentVelocity > 0 ? 1 : -1);

    if(abs(distanceToGo) < 100)
        profile.targetVelocity = 0;

    serialLog((char *)"Motion profile is outputting: ", 3);
    serialLog(float(profile.targetVelocity), 3);
    serialLog((char *)",", 3);

    serialLog((char *)"Target Position: ", 3);
    serialLog(float(profile.targetPosition), 3);
    serialLog((char *)",", 3);

    serialLog((char *)"Stopping position: ", 3);
    serialLog(float(stoppingDistance), 3);
    serialLogln((char *)",", 3);

    return profile.targetVelocity;
}
