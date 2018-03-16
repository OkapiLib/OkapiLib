/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_VELINTEGRATEDCONTROLLER_HPP_
#define _OKAPI_VELINTEGRATEDCONTROLLER_HPP_

#include "okapi/control/async/asyncVelocityController.hpp"
#include "okapi/device/abstractMotor.hpp"
#include "okapi/device/integratedEncoder.hpp"

namespace okapi {
class VelIntegratedControllerParams : public AsyncVelocityControllerParams {
  public:
  VelIntegratedControllerParams(const AbstractMotor &imotor) : motor(imotor) {
  }

  const AbstractMotor &motor;
};

/**
 * Closed-loop controller that uses the V5 motor's onboard control to move.
 */
class VelIntegratedController : public AsyncVelocityController {
  public:
  VelIntegratedController(const AbstractMotor &imotor) : motor(imotor), lastTarget(0) {
  }

  VelIntegratedController(const VelIntegratedControllerParams &iparams)
    : motor(iparams.motor), lastTarget(0) {
  }

  virtual ~VelIntegratedController() = default;

  /**
   * Sets the target for the controller.
   */
  virtual void setTarget(const double itarget) {
    motor.move_velocity(itarget);
    lastTarget = itarget;
  }

  /**
   * Returns the last error of the controller.
   */
  virtual double getError() const {
    return lastTarget - motor.get_actual_velocity();
  }

  /**
   * Resets the controller so it can start from 0 again properly. Keeps configuration from
   * before.
   */
  virtual void reset() {
    // Don't have to do anything
  }

  private:
  const AbstractMotor &motor;
  double lastTarget;
};
} // namespace okapi

#endif
