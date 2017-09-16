#ifndef OKAPI_NSPID
#define OKAPI_NSPID

#include "control/pid.h"
#include "control/velocity.h"

namespace okapi {
  class NsPid : public Pid {
    public:
      using Pid::Pid;

      /**
       * Do one iteration of the controller
       * @param  inewReading New measurement
       * @return            Controller output
       */
      virtual float loop(const float inewReading);
    protected:
      VelMath velMath;
      static constexpr float minVel = 1;
  };
}

#endif /* end of include guard: OKAPI_NSPID */
