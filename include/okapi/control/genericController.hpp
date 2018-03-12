/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_GENERICCONTROLLER_HPP_
#define _OKAPI_GENERICCONTROLLER_HPP_

#include <memory>
#include <array>
#include "okapi/control/controlObject.hpp"
#include "okapi/device/motor.hpp"

namespace okapi {
  template<size_t motorNum>
  class GenericController {
  public:
    GenericController(const std::array<Motor, motorNum> &imotorList, const std::shared_ptr<ControlObject> &iptr):
      motors(imotorList),
      controller(iptr) {}

    void step(const float ireading) {
      controller->step(ireading);
      for (size_t i = 0; i < motors.size(); i++)
        motors[i].setTS(static_cast<int>(controller->getOutput()));
    }

    void setTarget(const float itarget) { controller->setTarget(itarget); }

    void getOutput() const { return controller->getOutput(); }

    void setSampleTime(const int isampleTime) { controller->setSampleTime(isampleTime); }

    void setOutputLimits(float imax, float imin) { controller->setOutputLimits(imax, imin); }

    void reset() { controller->reset(); }

    void flipDisable() { controller->flipDisable(); }

  private:
    std::array<Motor, motorNum> motors;
    std::shared_ptr<ControlObject> controller;
  };
}

#endif
