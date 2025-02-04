#ifndef CHESSBOT_TIMER_CPP
#define CHESSBOT_TIMER_CPP

#include "utils/timer.h"

#include "Arduino.h"
#include "utils/logging.h"

#include <vector>

// How to pass function from within a class (Uses a lambda that captures the class' pointer)
// [this](){ func(); }

namespace ChessBot
{
    std::vector<Timer> timers;

    // Will run the function you provide after 'delay' amount of milliseconds
    Timer timerDelay(int delay, TimerCallback func) {
        Timer newTimer;
        newTimer.oneOff = true;
        newTimer.delay = delay;
        newTimer.lastMillis = millis();
        newTimer.func = func;
        timers.push_back(newTimer);
        return newTimer;
    }

    // Last I checked, this one didn't actually work...
    // Just uhh, be careful using it I guess. Probably has a bug
    Timer timerInterval(int interval, TimerCallback func) {
        Timer newTimer;
        newTimer.oneOff = false;
        newTimer.delay = interval;
        newTimer.lastMillis = millis();
        newTimer.func = func;
        timers.push_back(newTimer);
        return newTimer;
    }

    // Deletes the timer from the vector without conserving order
    // (Copies last item replacing index, then pops last item)
    void timerCancel(size_t index) {
        if (index <= timers.size()) {
            timers[index] = timers.back();
            timers.pop_back();
        }
    }

    void timerCancelAll() {
        timers.clear();
    }

    void timerStep() {
        for (size_t i = 0; i < timers.size();) {
            if (millis() - timers[i].lastMillis >= timers[i].delay) {
                TimerCallback func = timers[i].func;
                if (timers[i].oneOff) {
                    timerCancel(i);
                } else {
                    timers[i].lastMillis = millis();
                    i++;
                }
                func();
            } else {
                i++;
            }
        }
    }
};

#endif