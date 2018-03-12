/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_VELOCITY_HPP_
#define _OKAPI_VELOCITY_HPP_

#include "okapi/filter/demaFilter.hpp"

namespace okapi {
  class VelMathParams {
    public:
      VelMathParams(const float iticksPerRev, const float ialpha = 0.19, const float ibeta = 0.041):
        ticksPerRev(iticksPerRev),
        alpha(ialpha),
        beta(ibeta) {}

      float ticksPerRev, alpha, beta;
  };

  class VelMath {
  public:
    VelMath(const float iticksPerRev, const float ialpha = 0.19, const float ibeta = 0.041):
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
    float step(const float inewPos);

    void setGains(const float ialpha, const float ibeta) { filter.setGains(ialpha, ibeta); }

    void setTicksPerRev(const float iTPR) { ticksPerRev = iTPR; }

    float getOutput() const { return vel; }

    float getDiff() const { return vel - lastVel; }

  private:
    long lastTime;
    float vel, lastVel, lastPos, ticksPerRev;
    DemaFilter filter;
  };
}

#endif
