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
class VelMathParams {
  public:
  VelMathParams(const double iticksPerRev, const double ialpha = 0.19, const double ibeta = 0.041);

  virtual ~VelMathParams();

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
   * @param iparams VelMathParams
   */
  VelMath(const VelMathParams &iparams);

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
  void setGains(const double ialpha, const double ibeta);

  /**
   * Set ticks per revolution (or whatever units you are using).
   *
   * @para iTPR ticks per revolution
   */
  void setTicksPerRev(const double iTPR);

  /**
   * Get the last calculated output.
   */
  double getOutput() const;

  /**
   * Get the difference between the last output and the output before that.
   */
  double getDiff() const;

  protected:
  uint32_t lastTime;
  double vel, lastVel, lastPos, ticksPerRev;
  DemaFilter filter;
};
} // namespace okapi

#endif
