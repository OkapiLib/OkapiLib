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
#include "okapi/filter/demaFilter.hpp"

namespace okapi {
class VelMathArgs {
  public:
  VelMathArgs(const double iticksPerRev, const double ialpha = 0.19, const double ibeta = 0.041);

  virtual ~VelMathArgs();

  const double ticksPerRev, alpha, beta;
};

class VelMath {
  public:
  /**
   * Velocity math helper. Calculates filtered velocity (DemaFilter).
   *
   * @param iticksPerRev number of ticks per revolution (or whatever units you are using)
   * @param ialpha alpha gain
   * @param ibeta beta gain
   */
  VelMath(const double iticksPerRev, const double ialpha = 0.19, const double ibeta = 0.041);

  /**
   * Velocity math helper. Calculates filtered velocity (DemaFilter).
   *
   * @param iparams VelMathArgs
   */
  VelMath(const VelMathArgs &iparams);

  /**
   * Calculate new velocity.
   *
   * @param inewPos new position
   * @return new velocity
   */
  virtual double step(const double inewPos);

  /**
   * Set filter gains.
   *
   * @param ialpha alpha gain
   * @param ibeta beta gain
   */
  virtual void setGains(const double ialpha, const double ibeta);

  /**
   * Set ticks per revolution (or whatever units you are using).
   *
   * @para iTPR ticks per revolution
   */
  virtual void setTicksPerRev(const double iTPR);

  /**
   * Get the last calculated output.
   */
  virtual double getOutput() const;

  /**
   * Get the acceleration.
   */
  virtual double getAccel() const;

  protected:
  uint32_t lastTime = 0;
  double vel = 0;
  double accel = 0;
  double lastVel = 0;
  double lastPos = 0;
  double ticksPerRev;
  DemaFilter filter;
};
} // namespace okapi

#endif
