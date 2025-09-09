#ifndef CHESSBOT_TIMER_H
#define CHESSBOT_TIMER_H

// Built-In Libraries
#include <vector>
//we're gonna need this if we need to use delays which require functions
#include <functional>

// This callback allows you to just pass in the pointer of a function as a parameter
using TimerCallback = std::function<void()>;
struct Timer {
    // Whether the timer is cancelled after it finishes, or set to run again
    bool oneOff;
    // The amount of time until the timer expires
    int delay;
    // The time that the timer started (or refreshed in the case of non one-off timers)
    unsigned long lastMillis;
    // The ID of the timer (Timestamp timer was created. Same as lastMillis for oneOffs)
    unsigned long id;
    // The function to be run after the timer expires
    TimerCallback func;
};

unsigned long timerDelay(int delay, TimerCallback func);
unsigned long timerInterval(int interval, TimerCallback func);

Timer getTimer(unsigned long id);

void resetTimer(unsigned long id);
void timerCancel(unsigned long id);
void timerCancelAll();

void timerStep();

#endif