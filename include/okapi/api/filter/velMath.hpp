/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/filter/composableFilter.hpp"
#include "okapi/api/units/QAngularAcceleration.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "okapi/api/units/QTime.hpp"
#include "okapi/api/util/abstractTimer.hpp"
#include "okapi/api/util/logging.hpp"
#include <memory>

namespace okapi {
class VelMath {
  public:
  /**
   * Velocity math helper. Calculates filtered velocity. Throws a std::invalid_argument exception
   * if iticksPerRev is zero.
   *
   * @param iticksPerRev number of ticks per revolution (or whatever units you are using)
   * @param ifilter filter used for filtering the calculated velocity
   * @param isampleTime the minimum time between velocity measurements
   * @param ilogger The logger this instance will log to.
   */
  VelMath(double iticksPerRev,
          std::unique_ptr<Filter> ifilter,
          QTime isampleTime,
          std::unique_ptr<AbstractTimer> iloopDtTimer,
          const std::shared_ptr<Logger> &ilogger = std::make_shared<Logger>());

  virtual ~VelMath();

  /**
   * Calculates the current velocity and acceleration. Returns the (filtered) velocity.
   *
   * @param inewPos new position
   * @return current (filtered) velocity
   */
  virtual QAngularSpeed step(double inewPos);

  /**
   * Sets ticks per revolution (or whatever units you are using).
   *
   * @para iTPR ticks per revolution
   */
  virtual void setTicksPerRev(double iTPR);

  /**
   * Returns the last calculated velocity.
   */
  virtual QAngularSpeed getVelocity() const;

  /**
   * Returns the last calculated acceleration.
   */
  virtual QAngularAcceleration getAccel() const;

  protected:
  std::shared_ptr<Logger> logger;
  QAngularSpeed vel;
  QAngularSpeed lastVel;
  QAngularAcceleration accel;
  double lastPos = 0;
  double ticksPerRev;

  QTime sampleTime;
  std::unique_ptr<AbstractTimer> loopDtTimer;
  std::unique_ptr<Filter> filter;
};
} // namespace okapi
