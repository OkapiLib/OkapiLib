/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_MOTORCONTROLLER_HPP_
#define _OKAPI_MOTORCONTROLLER_HPP_

#include "api.h"
#include "okapi/control/controlObject.hpp"
#include "okapi/device/abstractMotor.hpp"
#include <array>
#include <memory>

namespace okapi {
class MotorController {
  public:
  MotorController(const AbstractMotor &imotor, ControlObject &iptr)
    : motor(imotor), controller(iptr) {
  }

  void step(const float ireading) {
    controller.step(ireading);
    motor.set_velocity(static_cast<int>(controller.getOutput()));
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
  const AbstractMotor &motor;
  ControlObject &controller;
};
} // namespace okapi

#endif
