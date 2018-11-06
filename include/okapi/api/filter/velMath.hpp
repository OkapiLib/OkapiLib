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
class VelMathArgs {
  public:
  explicit VelMathArgs(double iticksPerRev, QTime isampleTime = 0_ms);
  VelMathArgs(double iticksPerRev,
              const std::shared_ptr<Filter> &ifilter,
              QTime isampleTime = 0_ms);

  virtual ~VelMathArgs();

  double ticksPerRev;
  std::shared_ptr<Filter> filter;
  QTime sampleTime;
};

class VelMath {
  public:
  /**
   * Velocity math helper. Calculates filtered velocity. Throws a std::invalid_argument exception
   * if iticksPerRev is zero.
   *
   * @param iticksPerRev number of ticks per revolution (or whatever units you are using)
   * @param ifilter filter used for filtering the calculated velocity
   * @param isampleTime the minimum time between velocity measurements
   */
  VelMath(double iticksPerRev,
          const std::shared_ptr<Filter> &ifilter,
          QTime isampleTime,
          std::unique_ptr<AbstractTimer> iloopDtTimer);

  VelMath(const VelMathArgs &iparams, std::unique_ptr<AbstractTimer> iloopDtTimer);

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
  Logger *logger;
  QAngularSpeed vel;
  QAngularSpeed lastVel;
  QAngularAcceleration accel;
  double lastPos = 0;
  double ticksPerRev;

  QTime sampleTime;
  std::unique_ptr<AbstractTimer> loopDtTimer;
  std::shared_ptr<Filter> filter;
};
} // namespace okapi
