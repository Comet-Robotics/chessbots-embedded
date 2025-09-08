#ifndef CHESSBOT_TIMER_CPP
#define CHESSBOT_TIMER_CPP

// Associated Header File
#include "utils/timer.h"

// Built-In Libraries
#include "Arduino.h"
#include <map>
#include <vector>

// Custom Libraries
#include "utils/logging.h"

// How to pass function from within a class (Uses a lambda that captures the class' pointer)
// [this](){ func(); }

std::map<unsigned long, Timer> timers;
// Timers that have expired and need to be deleted are added to this
std::vector<unsigned long> choppingBlock;

// Will run the function you provide after 'delay' amount of milliseconds.
// Accepts a delay in milliseconds, and a function as a pointer
// Returns the id of the timer
unsigned long timerDelay(int delay, TimerCallback func) {
    // This uses a custom Timer struct (located in timer.h)
    Timer newTimer;
    newTimer.id = micros();
    newTimer.oneOff = true;
    newTimer.delay = delay;
    newTimer.func = func;
    newTimer.lastMillis = millis();
    timers[newTimer.id] = newTimer;
    // Returns the id for modifying
    return newTimer.id;
}

// Will run the function you provide every 'interval' milliseconds.
// Accepts an interval in milliseconds, and a function as a pointer
// Returns the id of the timer
unsigned long timerInterval(int interval, TimerCallback func) {
    // This uses a custom Timer struct (located in timer.h)
    Timer newTimer;
    newTimer.id = micros();
    newTimer.oneOff = false;
    newTimer.delay = interval;
    newTimer.func = func;
    newTimer.lastMillis = millis();
    timers[newTimer.id] = newTimer;
    // Returns the id for modifying
    return newTimer.id;
}

// Returns the Timer struct associated with the id
Timer getTimer(unsigned long id) {
    return timers[id];
}

// Resets the time on an active timer
void resetTimer(unsigned long id) {
    timers[id].lastMillis = millis();
}

// Deletes the timer from the map
void timerCancel(unsigned long id) {
    timers.erase(id);
}

// Cancels all timers
void timerCancelAll() {
    timers.clear();
}

// Checks timers for any expired ones
void timerStep() {
    // Iterates through the timer map key value pairs
    for (auto index = timers.begin(); index != timers.end(); index++) {
        unsigned long id = index->first;
        if (millis() - timers[id].lastMillis >= timers[id].delay) {
            TimerCallback func = timers[id].func;
            if (timers[id].oneOff) {
                // If the timer is one-off, add it to a list
                choppingBlock.push_back(id);
            } else {
                // If the timer isn't one-off, refresh it after it expires
                resetTimer(id);
            }

            // Calls the linked function
            func();
        }
    }

    // Cancel all the expired timers in the list
    for (int i = 0; i < choppingBlock.size(); i++) {
        timerCancel(choppingBlock.back());
        choppingBlock.pop_back();
    }
}

#endif