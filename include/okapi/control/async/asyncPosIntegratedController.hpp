/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ASYNCPOSINTEGRATEDCONTROLLER_HPP_
#define _OKAPI_ASYNCPOSINTEGRATEDCONTROLLER_HPP_

#include "okapi/control/async/asyncPositionController.hpp"
#include "okapi/control/util/settledUtil.hpp"
#include "okapi/device/motor/motor.hpp"
#include "okapi/device/motor/motorGroup.hpp"

namespace okapi {
class AsyncPosIntegratedController;

class AsyncPosIntegratedControllerArgs : public AsyncPositionControllerArgs {
  public:
  AsyncPosIntegratedControllerArgs(std::shared_ptr<AbstractMotor> imotor);

  std::shared_ptr<AbstractMotor> motor;
};

/**
 * Closed-loop controller that uses the V5 motor's onboard control to move.
 */
class AsyncPosIntegratedController : public AsyncPositionController {
  public:
  AsyncPosIntegratedController(Motor imotor);

  AsyncPosIntegratedController(MotorGroup imotor);

  AsyncPosIntegratedController(std::shared_ptr<AbstractMotor> imotor);

  AsyncPosIntegratedController(const AsyncPosIntegratedControllerArgs &iparams);

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

  protected:
  std::shared_ptr<AbstractMotor> motor;
  double lastTarget = 0;
  SettledUtil settledUtil;
};
} // namespace okapi

#endif
