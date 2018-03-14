/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_MOTORCONTROLLER_HPP_
#define _OKAPI_MOTORCONTROLLER_HPP_

#include "api.h"
#include "okapi/control/iterativeController.hpp"
#include "okapi/control/velocityDomainController.hpp"
#include "okapi/device/abstractMotor.hpp"
#include <array>
#include <memory>

namespace okapi {
class MotorController : public VelocityDomainController {
  public:
  MotorController(const AbstractMotor &imotor, IterativeController &iptr)
    : motor(imotor), controller(iptr) {
  }

  double step(const double ireading) {
    controller.step(ireading);
    motor.move_velocity(static_cast<int>(controller.getOutput()));
    return controller.getOutput();
  }

  void setTarget(const double itarget) {
    controller.setTarget(itarget);
  }

  double getOutput() const {
    return controller.getOutput();
  }

  double getError() const {
    return controller.getError();
  }

  void setSampleTime(const int isampleTime) {
    controller.setSampleTime(isampleTime);
  }

  void setOutputLimits(double imax, double imin) {
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
  IterativeController &controller;
};
} // namespace okapi

#endif
