/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_NSPID_HPP_
#define _OKAPI_NSPID_HPP_

#include "okapi/control/pid.hpp"
#include "okapi/control/velMath.hpp"

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

#endif
