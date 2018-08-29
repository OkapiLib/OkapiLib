/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_TIMER_HPP_
#define _OKAPI_TIMER_HPP_

#include "okapi/api/util/abstractTimer.hpp"

namespace okapi {
class Timer : public AbstractTimer {
  public:
  Timer();

  /**
   * Returns the current time in units of QTime.
   *
   * @return the current time
   */
  QTime millis() const override;

  /**
   * Returns the time passed in ms since the previous call of this function.
   *
   * @return The time passed in ms since the previous call of this function
   */
  QTime getDt() override;

  /**
   * Returns the time passed in ms since the previous call of getDt(). Does not change the time
   * recorded by getDt().
   *
   * @return The time passed in ms since the previous call of getDt()
   */
  QTime readDt() override;

  /**
   * Returns the time the timer was first constructed.
   *
   * @return The time the timer was first constructed
   */
  QTime getStartingTime() const override;

  /**
   * Returns the time since the timer was first constructed.
   *
   * @return The time since the timer was first constructed
   */
  QTime getDtFromStart() const override;

  /**
   * Place a time marker. Placing another marker will overwrite the previous one.
   */
  void placeMark() override;

  /**
   * Clears the marker.
   *
   * @return The old marker
   */
  QTime clearMark() override;

  /**
   * Place a hard time marker. Placing another hard marker will not overwrite the previous one;
   * instead, call clearHardMark() and then place another.
   */
  void placeHardMark() override;

  /**
   * Clears the hard marker.
   *
   * @return The old hard marker
   */
  QTime clearHardMark() override;

  /**
   * Returns the time since the time marker.
   *
   * @return The time since the time marker
   */
  QTime getDtFromMark() const override;

  /**
   * Returns the time since the hard time marker.
   *
   * @return The time since the hard time marker
   */
  QTime getDtFromHardMark() const override;

  /**
   * Returns true when the input time period has passed, then resets. Meant to be used in loops
   * to run an action every time period without blocking.
   *
   * @param time time period
   * @return true when the input time period has passed, false after reading true until the
   *   period has passed again
   */
  bool repeat(QTime time) override;

  /**
   * Returns true when the input time period has passed, then resets. Meant to be used in loops
   * to run an action every time period without blocking.
   *
   * @param frequency the repeat frequency
   * @return true when the input time period has passed, false after reading true until the
   *   period has passed again
   */
  bool repeat(QFrequency frequency) override;

  protected:
  QTime firstCalled;
  QTime lastCalled;
  QTime mark;
  QTime hardMark;
  QTime repeatMark;
};
} // namespace okapi

#endif
