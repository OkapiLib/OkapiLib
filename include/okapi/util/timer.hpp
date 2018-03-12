/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_TIMER_HPP_
#define _OKAPI_TIMER_HPP_

#include "api.h"

namespace okapi {
  class Timer {
  public:
    Timer():
      firstCalled(pros::millis()),
      lastCalled(0),
      mark(0),
      hardMark(0),
      repeatMark(0) {}

    /**
     * Returns the time passed in ms since the previous call of this function.
     * 
     * @return The time passed in ms since the previous call of this function
     */
    uint32_t getDt();

    /**
     * Returns the time the timer was first initialized.
     * 
     * @return The time the timer was first initialized
     */
    uint32_t getStartingTime() const;

    /**
     * Returns the time since the timer was first initialized.
     * 
     * @return The time since the timer was first initialized
     */
    uint32_t getDtFromStart() const;

    /**
     * Place a time marker. Placing another marker will overwrite the previous one.
     */
    void placeMark();

    /**
     * Place a hard time marker. Placing another hard marker will not overwrite the previous one;
     * instead, call clearHardMark() and then place another.
     */
    void placeHardMark();

    /**
     * Clears the hard marker.
     * 
     * @return The old hard marker
     */
    uint32_t clearHardMark();

    /**
     * Returns the time since the time marker.
     * 
     * @return The time since the time marker
     */
    uint32_t getDtFromMark() const;

    /**
     * Returns the time since the hard time marker.
     * 
     * @return The time since the hard time marker
     */
    uint32_t getDtFromHardMark() const;

    /**
     * Returns true when the input time period has passed, then resets. Meant to be used in loops
     * to run an action every so many ms without blocking.
     * 
     * @param  ms Time period
     * @return    True when the input time period has passed, false after
     *                 reading true until the period has passed again
     */
    bool repeat(uint32_t ms);

  private:
    uint32_t firstCalled, lastCalled, mark, hardMark, repeatMark;
  };
}

#endif
