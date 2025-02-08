#ifndef CHESSBOT_TIMER_H
#define CHESSBOT_TIMER_H

// Built-In Libraries
#include <vector>

namespace ChessBot
{
    // This callback allows you to just pass in the pointer of a function as a parameter
    using TimerCallback = void (*)();
    struct Timer {
        // Whether the timer is cancelled after it finishes, or set to run again
        bool oneOff;
        // The amount of time until the timer expires
        int delay;
        // The time that the timer started (or refreshed in the case of non one-off timers)
        unsigned long lastMillis;
        // The function to be run after the timer expires
        TimerCallback func;
    };

    size_t timerDelay(int delay, TimerCallback func);
    size_t timerInterval(int interval, TimerCallback func);

    void timerCancel(size_t index);
    void timerCancelAll();

    void timerStep();
};

#endif