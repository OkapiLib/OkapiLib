/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_TIMER_HPP_
#define _OKAPI_TIMER_HPP_

#include "api.h"
#include "okapi/units/QFrequency.hpp"
#include "okapi/units/QTime.hpp"

namespace okapi {
class Timer {
  public:
  Timer();

  virtual ~Timer();

  /**
   * Returns the time passed in ms since the previous call of this function.
   *
   * @return The time passed in ms since the previous call of this function
   */
  virtual QTime getDt();

  /**
   * Returns the time the timer was first constructed.
   *
   * @return The time the timer was first constructed
   */
  virtual QTime getStartingTime() const;

  /**
   * Returns the time since the timer was first constructed.
   *
   * @return The time since the timer was first constructed
   */
  virtual QTime getDtFromStart() const;

  /**
   * Place a time marker. Placing another marker will overwrite the previous one.
   */
  virtual void placeMark();

  /**
   * Place a hard time marker. Placing another hard marker will not overwrite the previous one;
   * instead, call clearHardMark() and then place another.
   */
  virtual void placeHardMark();

  /**
   * Clears the hard marker.
   *
   * @return The old hard marker
   */
  virtual QTime clearHardMark();

  /**
   * Returns the time since the time marker.
   *
   * @return The time since the time marker
   */
  virtual QTime getDtFromMark() const;

  /**
   * Returns the time since the hard time marker.
   *
   * @return The time since the hard time marker
   */
  virtual QTime getDtFromHardMark() const;

  /**
   * Returns true when the input time period has passed, then resets. Meant to be used in loops
   * to run an action every time period without blocking.
   *
   * @param time time period
   * @return true when the input time period has passed, false after reading true until the
   *   period has passed again
   */
  virtual bool repeat(const QTime time);

  /**
   * Returns true when the input time period has passed, then resets. Meant to be used in loops
   * to run an action every time period without blocking.
   *
   * @param frequency the repeat frequency
   * @return true when the input time period has passed, false after reading true until the
   *   period has passed again
   */
  virtual bool repeat(const QFrequency frequency);

  protected:
  QTime firstCalled;
  QTime lastCalled;
  QTime mark;
  QTime hardMark;
  QTime repeatMark;

  static QTime millis();
};
} // namespace okapi

#endif
