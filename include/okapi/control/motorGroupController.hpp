/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_MOTORGROUPCONTROLLER_HPP_
#define _OKAPI_MOTORGROUPCONTROLLER_HPP_

#include "api.h"
#include "okapi/control/controlObject.hpp"
#include <array>
#include <memory>

namespace okapi {
template <size_t motorNum> class MotorGroupController {
  public:
  MotorGroupController(const std::array<pros::Motor, motorNum> &imotorList, ControlObject &iptr)
    : motors(imotorList), controller(iptr) {
  }

  void step(const float ireading) {
    controller.step(ireading);
    for (size_t i = 0; i < motors.size(); i++)
      motors[i].set_velocity(static_cast<int>(controller.getOutput()));
  }

  void setTarget(const float itarget) {
    controller.setTarget(itarget);
  }

  float getOutput() const {
    return controller.getOutput();
  }

  float getError() const {
    return controller.getError();
  }

  void setSampleTime(const int isampleTime) {
    controller.setSampleTime(isampleTime);
  }

  void setOutputLimits(float imax, float imin) {
    controller.setOutputLimits(imax, imin);
  }

  void reset() {
    controller.reset();
  }

  void flipDisable() {
    controller.flipDisable();
  }

  private:
  std::array<pros::Motor, motorNum> motors;
  ControlObject &controller;
};
} // namespace okapi

#endif
