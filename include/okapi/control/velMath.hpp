/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_VELOCITY_HPP_
#define _OKAPI_VELOCITY_HPP_

#include "okapi/filter/demaFilter.hpp"

namespace okapi {
  class VelMathParams {
    public:
      VelMathParams(const double iticksPerRev, const double ialpha = 0.19, const double ibeta = 0.041):
        ticksPerRev(iticksPerRev),
        alpha(ialpha),
        beta(ibeta) {}

      const double ticksPerRev, alpha, beta;
  };

  class VelMath {
  public:
    VelMath(const double iticksPerRev, const double ialpha = 0.19, const double ibeta = 0.041):
      lastTime(0),
      vel(0),
      lastVel(0),
      lastPos(0),
      ticksPerRev(iticksPerRev),
      filter(ialpha, ibeta) {}

    VelMath(const VelMathParams& iparams):
      lastTime(0),
      vel(0),
      lastVel(0),
      lastPos(0),
      ticksPerRev(iparams.ticksPerRev),
      filter(iparams.alpha, iparams.beta) {}

    /**
     * Calculate new velocity.
     * 
     * @param  inewPos New position
     * @return         New velocity
     */
    double step(const double inewPos);

    void setGains(const double ialpha, const double ibeta) { filter.setGains(ialpha, ibeta); }

    void setTicksPerRev(const double iTPR) { ticksPerRev = iTPR; }

    double getOutput() const { return vel; }

    double getDiff() const { return vel - lastVel; }

  private:
    long lastTime;
    double vel, lastVel, lastPos, ticksPerRev;
    DemaFilter filter;
  };
}

#endif
