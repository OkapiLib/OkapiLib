/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_VELOCITY_HPP_
#define _OKAPI_VELOCITY_HPP_

#include "api.h"
#include "okapi/filter/composableFilter.hpp"
#include "okapi/util/timer.hpp"
#include <memory>

namespace okapi {
class VelMathArgs {
  public:
  VelMathArgs(const double iticksPerRev);
  VelMathArgs(const double iticksPerRev, std::shared_ptr<Filter> ifilter);

  virtual ~VelMathArgs();

  const double ticksPerRev;
  std::shared_ptr<Filter> filter;
};

class VelMath {
  public:
  /**
   * Velocity math helper. Calculates filtered velocity. Filters using a 3-tap median filter
   * and a 5-tap averaging filter.
   *
   * @param iticksPerRev number of ticks per revolution (or whatever units you are using)
   */
  VelMath(const double iticksPerRev);

  /**
   * Velocity math helper. Calculates filtered velocity.
   *
   * @param iticksPerRev number of ticks per revolution (or whatever units you are using)
   * @param ifilter filter to use for filtering the velocity
   */
  VelMath(const double iticksPerRev, std::shared_ptr<Filter> ifilter);

  VelMath(const VelMathArgs &iparams);

  /**
   * This constructor is meant for unit testing.
   */
  VelMath(const double iticksPerRev, std::shared_ptr<Filter> ifilter,
          std::unique_ptr<Timer> iloopDtTimer);

  virtual ~VelMath();

  /**
   * Calculates the current velocity and acceleration. Returns the (filtered) velocity.
   *
   * @param inewPos new position
   * @return current (filtered) velocity
   */
  virtual double step(const double inewPos);

  /**
   * Sets ticks per revolution (or whatever units you are using).
   *
   * @para iTPR ticks per revolution
   */
  virtual void setTicksPerRev(const double iTPR);

  /**
   * Returns the last calculated velocity.
   */
  virtual double getVelocity() const;

  /**
   * Returns the last calculated acceleration.
   */
  virtual double getAccel() const;

  protected:
  double vel = 0;
  double accel = 0;
  double lastVel = 0;
  double lastPos = 0;
  double ticksPerRev;

  std::unique_ptr<Timer> loopDtTimer;
  std::shared_ptr<Filter> filter;
};
} // namespace okapi

#endif
