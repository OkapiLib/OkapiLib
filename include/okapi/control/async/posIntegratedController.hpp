/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_POSINTEGRATEDCONTROLLER_HPP_
#define _OKAPI_POSINTEGRATEDCONTROLLER_HPP_

#include "okapi/control/async/asyncPositionController.hpp"
#include "okapi/device/abstractMotor.hpp"

namespace okapi {
class PosIntegratedController;

class PosIntegratedControllerParams : public AsyncPositionControllerParams {
  public:
  PosIntegratedControllerParams(const AbstractMotor &imotor);

  std::shared_ptr<AsyncPositionController> make() const override;

  const AbstractMotor &motor;
};

/**
 * Closed-loop controller that uses the V5 motor's onboard control to move.
 */
class PosIntegratedController : public AsyncPositionController {
  public:
  PosIntegratedController(const AbstractMotor &imotor);

  PosIntegratedController(const PosIntegratedControllerParams &iparams);

  virtual ~PosIntegratedController();

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
  double lastTarget, offset;
};
} // namespace okapi

#endif
