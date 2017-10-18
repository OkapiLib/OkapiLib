#ifndef OKAPI_TIMER
#define OKAPI_TIMER

#include <API.h>

namespace okapi {
  class Timer {
  public:
    Timer():
      firstCalled(millis()),
      lastCalled(0),
      mark(0),
      hardMark(-1),
      repeatMark(-1) {}

    /**
     * Returns the time passed in ms since the previous call of this function
     * @return The time passed in ms since the previous call of this function
     */
    unsigned long getDt();

    /**
     * Returns the time the timer was first initialized
     * @return The time the timer was first initialized
     */
    unsigned long getStartingTime() const;

    /**
     * Returns the time since the timer was first initialized
     * @return The time since the timer was first initialized
     */
    unsigned long getDtFromStart() const;

    /**
     * Place a time marker. Placing another marker will overwrite the previous
     * one
     */
    void placeMark();

    /**
     * Place a hard time marker. Placing another hard marker will not overwrite
     * the previous one; instead, call clearHardMark() and then place another
     */
    void placeHardMark();

    /**
     * Clears the hard marker
     * @return The old hard marker
     */
    unsigned long clearHardMark();

    /**
     * Returns the time since the time marker
     * @return The time since the time marker
     */
    unsigned long getDtFromMark() const;

    /**
     * Returns the time since the hard time marker
     * @return The time since the hard time marker
     */
    unsigned long getDtFromHardMark() const;

    /**
     * Returns true when the input time period has passed, then resets. Meant to
     * be used in loops to run an action every so many ms without blocking
     * @param  ms Time period
     * @return    True when the input time period has passed, false after
     *                 reading true until the period has passed again
     */
    bool repeat(unsigned long ms);
  private:
    long firstCalled, lastCalled, mark, hardMark, repeatMark; //Long so we can use -1 even though millis() returns unsigned long
  };
}

#endif /* end of include guard: OKAPI_TIMER */
