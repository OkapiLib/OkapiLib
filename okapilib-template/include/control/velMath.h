#ifndef OKAPI_VELOCITY
#define OKAPI_VELOCITY

#include "filter/demaFilter.h"

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
     * Calculate new velocity
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

#endif /* end of include guard: OKAPI_VELOCITY */
