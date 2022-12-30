/*
Adapted from virtual delay library by: Albert van Dalen
*/
#include <Arduino.h>

class NonBlockingDelay {
public:
    NonBlockingDelay(unsigned long (*timerFunctionPtr)() = millis);
    void start(signed long delay);
    void repeat(signed long delay, void (*callBackPtr)());
    bool finished();

    bool running = 0;
    unsigned long timeOut, (*timerFunctionPtr)();
};

NonBlockingDelay::NonBlockingDelay(
    unsigned long (*timerFunctionPtr)())
    : timerFunctionPtr(timerFunctionPtr)
{
}

void NonBlockingDelay::repeat(signed long delay, void (*callBackPtr)())
{
      NonBlockingDelay::start(delay);
      if (NonBlockingDelay::finished()) {
        callBackPtr();
      }
}

void NonBlockingDelay::start(signed long delay)
{
    if (!running) {
        running = 1;
        timeOut = (*timerFunctionPtr)() + abs(delay);
    }
}

bool NonBlockingDelay::finished()
{
    if (running) {
        if ((signed long)((*timerFunctionPtr)() - timeOut) >= 0) {
            running = 0;
            return 1;
        }
    }
    return 0; 
}
