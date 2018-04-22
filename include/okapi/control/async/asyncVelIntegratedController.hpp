/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ASYNCVELINTEGRATEDCONTROLLER_HPP_
#define _OKAPI_ASYNCVELINTEGRATEDCONTROLLER_HPP_

#include "okapi/control/async/asyncVelocityController.hpp"
#include "okapi/control/util/settledUtil.hpp"
#include "okapi/device/motor/abstractMotor.hpp"
#include "okapi/device/motor/motor.hpp"
#include "okapi/device/motor/motorGroup.hpp"
#include "okapi/device/rotarysensor/integratedEncoder.hpp"
#include <memory>

namespace okapi {
class AsyncVelIntegratedControllerArgs : public AsyncVelocityControllerArgs {
  public:
  AsyncVelIntegratedControllerArgs(std::shared_ptr<AbstractMotor> imotor);

  std::shared_ptr<AbstractMotor> motor;
};

/**
 * Closed-loop controller that uses the V5 motor's onboard control to move. Input units are whatever
 * units the motor is in.
 */
class AsyncVelIntegratedController : public AsyncVelocityController {
  public:
  AsyncVelIntegratedController(Motor imotor);

  AsyncVelIntegratedController(MotorGroup imotor);

  AsyncVelIntegratedController(std::shared_ptr<AbstractMotor> imotor);

  AsyncVelIntegratedController(const AsyncVelIntegratedControllerArgs &iparams);

  /**
   * Sets the target for the controller.
   */
  virtual void setTarget(const double itarget) override;

  /**
   * Returns the last error of the controller.
   */
  virtual double getError() const override;

  /**
   * Returns whether the controller has settled at the target. Determining what settling means is
   * implementation-dependent.
   *
   * @return whether the controller is settled
   */
  virtual bool isSettled() override;

  /**
   * Resets the controller so it can start from 0 again properly. Keeps configuration from
   * before.
   */
  virtual void reset() override;

  /**
   * Changes whether the controller is off or on. Turning the controller on after it was off will
   * cause the controller to move to its last set target, unless it was reset in that time.
   */
  virtual void flipDisable() override;

  /**
   * Sets whether the controller is off or on. Turning the controller on after it was off will
   * cause the controller to move to its last set target, unless it was reset in that time.
   *
   * @param iisDisabled whether the controller is disabled
   */
  virtual void flipDisable(const bool iisDisabled) override;

  /**
   * Returns whether the controller is currently disabled.
   *
   * @return whether the controller is currently disabled
   */
  virtual bool isDisabled() const override;

  protected:
  std::shared_ptr<AbstractMotor> motor;
  double lastTarget = 0;
  bool controllerIsDisabled = false;
  bool hasFirstTarget = false;
  SettledUtil settledUtil;

  virtual void resumeMovement();
};
} // namespace okapi

#endif
