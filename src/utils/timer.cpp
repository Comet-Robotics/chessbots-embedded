#ifndef CHESSBOT_TIMER_CPP
#define CHESSBOT_TIMER_CPP

// Associated Header File
#include "utils/timer.h"

// Built-In Libraries
#include "Arduino.h"
#include <vector>

// Custom Libraries
#include "utils/logging.h"

// How to pass function from within a class (Uses a lambda that captures the class' pointer)
// [this](){ func(); }

namespace ChessBot
{
    std::vector<Timer> timers;

    // Will run the function you provide after 'delay' amount of milliseconds.
    // Accepts a delay in milliseconds, and a function as a pointer
    // Returns the id of the timer
    size_t timerDelay(int delay, TimerCallback func) {
        // This uses a custom Timer struct (located in timer.h)
        Timer newTimer;
        newTimer.oneOff = true;
        newTimer.delay = delay;
        newTimer.lastMillis = millis();
        newTimer.func = func;
        timers.push_back(newTimer);
        // Returns the index of the new timer within the timers vector. This is used as the id
        return timers.size()-1;
    }

    // Last I checked, this one didn't actually work...
    // Just uhh, be careful using it I guess. Probably has a bug
    // Accepts an interval in milliseconds, and a function as a pointer
    // Returns the id of the timer
    size_t timerInterval(int interval, TimerCallback func) {
        // This uses a custom Timer struct (located in timer.h)
        Timer newTimer;
        newTimer.oneOff = false;
        newTimer.delay = interval;
        newTimer.lastMillis = millis();
        newTimer.func = func;
        timers.push_back(newTimer);
        // Returns the index of the new timer within the timers vector. This is used as the id
        return timers.size()-1;
    }

    // Deletes the timer from the vector without conserving order
    // (Overwrites the timer at index 'id' with the vector's last item, then pops (deletes) last item)
    // We do it this way to avoid storing expired items that bloat the vector's size
    void timerCancel(size_t id) {
        if (id <= timers.size()) {
            timers[id] = timers.back();
            timers.pop_back();
        }
    }

    // Cancels all timers
    void timerCancelAll() {
        timers.clear();
    }

    // Checks timers for any expired ones
    void timerStep() {
        for (size_t index = 0; index < timers.size();) {
            // Checks if the timer
            if (millis() - timers[index].lastMillis >= timers[index].delay) {
                TimerCallback func = timers[index].func;
                if (timers[index].oneOff) {
                    // If the timer is one-off, cancel it after it expires
                    timerCancel(index);
                } else {
                    // If the timer isn't one-off, refresh it after it expires
                    timers[index].lastMillis = millis();
                    index++;
                }

                // Calls the linked function
                func();
            } else {
                index++;
            }
        }
    }
};

#endif