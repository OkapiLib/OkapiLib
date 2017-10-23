#ifndef OKAPI_NSPID
#define OKAPI_NSPID

#include "control/pid.h"
#include "control/velMath.h"

namespace okapi {
  class NsPid : public Pid {
    public:
      NsPid(const PidParams& iparams, const VelMathParams& ivelParams, const float iminVel, const float iscale = 0.1):
        Pid::Pid(iparams),
        velMath(ivelParams),
        minVel(iminVel),
        scale(iscale) {}

      /**
       * Do one iteration of the controller
       * @param  inewReading New measurement
       * @return            Controller output
       */
      virtual float step(const float inewReading) override;
    protected:
      VelMath velMath;
      float minVel, scale;
  };
}

#endif /* end of include guard: OKAPI_NSPID */
