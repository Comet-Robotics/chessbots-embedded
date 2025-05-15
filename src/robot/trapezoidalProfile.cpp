#include "robot/trapezoidalProfile.h"
#include "utils/logging.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include "../env.h"

using namespace std;

double updateTrapezoidalProfile(MotionProfile &profile, double dt, int8_t framesUntilprint, int critRange) {
    // distanceToGo = positive, you're still behind the target. || distanceToGo = negative, you're ahead.
    double distanceToGo = profile.targetPosition - profile.currentPosition;

    double changeInVelocity = 0;
    double stoppingDistance;

    //store all the absolute value versions so that we don't waste more computation calculating them when we already have before
    double absVelocity = fabs(profile.currentVelocity);
    double absDistanceToGo = fabs(distanceToGo);
    double absCurrentPos = fabs(profile.currentPosition);
    double absTargetVelocity = fabs(profile.targetVelocity);
    double absTargetPos = fabs(profile.targetPosition);
    
    // Formula is v^2 / 2a
    stoppingDistance = (profile.currentVelocity * profile.currentVelocity) / (2.0 * profile.maxAcceleration);
    
    // Acceleration phase. If we aren't already at max and we still have distance left to go, add a bit more velocity on.
    if (absVelocity < profile.maxVelocity && absDistanceToGo > 0)
        changeInVelocity = profile.maxAcceleration * dt * (distanceToGo > 0 ? 1 : -1);

    // Deceleration if we’re getting close to target. See first if we stop right now, will we overshoot our target? If so, begin decelerating
    if (absDistanceToGo <= stoppingDistance){
        float fraction;
        //if they're the same sign (or one is 0)
        if(profile.targetPosition * profile.currentPosition >= 0)
        {
            //no matter if theyr'e either both positive or both negative, it'll be between 0 and 1. You can verify this to make sure.
            if(absTargetPos > absCurrentPos)
            {
                
                fraction = profile.currentPosition / profile.targetPosition;
            }
            else
            {
                fraction = profile.targetPosition / profile.currentPosition;
            }
        }
        //either the target or start position is negative, the other is positive
        else
        {
            //going to have it as target / range. So liike if going from 50 to -150, that'd be 150/200. It's not really right, but it's kinda
            //finnicky how we should be handling it. Should check with people smarter than me to make sure
            float range = absTargetPos + absCurrentPos;
            fraction = profile.targetPosition / range;
        }
        //making it dependent on leftover distance/dt instead of maxAcceleration*dt
        changeInVelocity = -(fabs(fraction)) * profile.maxAcceleration * dt * (profile.currentVelocity > 0 ? 1 : -1);

        //want to make sure that we're not going to be decelerating so hard that we just reverse ourselves and don't reach our target. So if our
        //change in velocity causes use to reverse direction, just set it so we will change our velocity to 0.
        if(fabs(changeInVelocity) > absTargetVelocity)
        {
            changeInVelocity = profile.targetVelocity * -1;
        }
    }

    profile.targetVelocity += changeInVelocity;

    //clip off velocity if it's too high
    if (absTargetVelocity > profile.maxVelocity)
        profile.targetVelocity = profile.maxVelocity * (profile.targetVelocity > 0 ? 1 : -1);

    //if we're getting close to our target, may as well just stop it. Note that the reason we have the min function
    //is because sometimes critRange will be an even smaller number when we're moving very smaller distances (like when
    //we turn for edge alignment)
    if(absDistanceToGo < min(critRange, 75))
        profile.targetVelocity = 0;

    //use macros so that if we're not even logging, we won't eeven upload the code
#if LOGGING_LEVEL >= 3
    if(framesUntilprint % 15 == 0)
    {
        serialLog("Change in velocity was: ", 3);
        serialLog(float(changeInVelocity), 3);
        serialLog(", ", 3);
        
        serialLog("Motion profile is outputting: ", 3);
        serialLog(float(profile.targetVelocity), 3);
        serialLog(", ", 3);

        serialLog("Current Position: ", 3);
        serialLog(float(profile.currentPosition), 3);
        serialLog(", ", 3);

        serialLog("Target Position: ", 3);
        serialLog(float(profile.targetPosition), 3);
        serialLog(", ", 3);

        serialLog("Stopping position: ", 3);
        serialLogln(float(stoppingDistance), 3);
    }

#endif

    return profile.targetVelocity;
}
