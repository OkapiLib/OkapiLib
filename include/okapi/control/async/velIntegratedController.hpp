/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_VELINTEGRATEDCONTROLLER_HPP_
#define _OKAPI_VELINTEGRATEDCONTROLLER_HPP_

#include "okapi/control/async/asyncVelocityController.hpp"
#include "okapi/device/motor/abstractMotor.hpp"
#include "okapi/device/rotarysensor/integratedEncoder.hpp"

namespace okapi {
class VelIntegratedController;

class VelIntegratedControllerParams : public AsyncVelocityControllerParams {
  public:
  VelIntegratedControllerParams(const AbstractMotor &imotor);

  virtual ~VelIntegratedControllerParams();

  std::shared_ptr<AsyncVelocityController> make() const override;

  const AbstractMotor &motor;
};

/**
 * Closed-loop controller that uses the V5 motor's onboard control to move.
 */
class VelIntegratedController : public AsyncVelocityController {
  public:
  VelIntegratedController(const AbstractMotor &imotor);

  VelIntegratedController(const VelIntegratedControllerParams &iparams);

  virtual ~VelIntegratedController();

  /**
   * Sets the target for the controller.
   */
  virtual void setTarget(const double itarget) override;

  /**
   * Returns the last error of the controller.
   */
  double getError() const override;

  /**
   * Resets the controller so it can start from 0 again properly. Keeps configuration from
   * before.
   */
  void reset() override;

  protected:
  const AbstractMotor &motor;
  double lastTarget;
};
} // namespace okapi

#endif
