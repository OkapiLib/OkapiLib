/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_POSINTEGRATEDCONTROLLER_HPP_
#define _OKAPI_POSINTEGRATEDCONTROLLER_HPP_

#include "okapi/control/asyncPositionController.hpp"
#include "okapi/device/abstractMotor.hpp"
#include "okapi/device/integratedEncoder.hpp"

namespace okapi {
class PosIntegratedControllerParams : public AsyncPositionControllerParams {
  public:
  PosIntegratedControllerParams(const AbstractMotor &imotor) : motor(imotor) {
  }

  const AbstractMotor &motor;
};

/**
 * Closed-loop controller that uses the V5 motor's onboard control to move.
 */
class PosIntegratedController : public AsyncPositionController {
  public:
  PosIntegratedController(const AbstractMotor &imotor) : motor(imotor), lastTarget(0) {
  }

  PosIntegratedController(const PosIntegratedControllerParams &iparams)
    : motor(iparams.motor), lastTarget(0) {
  }

  virtual ~PosIntegratedController() = default;

  /**
   * Sets the target for the controller.
   */
  virtual void setTarget(const double itarget) {
    motor.move_absolute(itarget, 100);
  }

  /**
   * Returns the last error of the controller.
   */
  virtual double getError() const {
    return lastTarget - motor.get_position();
  }

  private:
  const AbstractMotor &motor;
  double lastTarget;
};
} // namespace okapi

#endif
