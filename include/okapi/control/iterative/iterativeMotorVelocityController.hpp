/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ITERATIVEMOTORVELOCITYCONTROLLER_HPP_
#define _OKAPI_ITERATIVEMOTORVELOCITYCONTROLLER_HPP_

#include "api.h"
#include "okapi/control/iterative/iterativeVelocityController.hpp"
#include "okapi/device/motor/abstractMotor.hpp"
#include "okapi/device/motor/motor.hpp"
#include "okapi/device/motor/motorGroup.hpp"
#include <array>
#include <memory>

namespace okapi {
class IterativeMotorVelocityControllerArgs : public IterativeVelocityControllerArgs {
  public:
  IterativeMotorVelocityControllerArgs(std::shared_ptr<AbstractMotor> imotor,
                                       std::shared_ptr<IterativeVelocityController> icontroller);

  std::shared_ptr<AbstractMotor> motor;
  std::shared_ptr<IterativeVelocityController> controller;
};

class IterativeMotorVelocityController : public IterativeVelocityController {
  public:
  IterativeMotorVelocityController(Motor imotor,
                                   std::shared_ptr<IterativeVelocityController> icontroller);

  IterativeMotorVelocityController(MotorGroup imotor,
                                   std::shared_ptr<IterativeVelocityController> icontroller);

  IterativeMotorVelocityController(std::shared_ptr<AbstractMotor> imotor,
                                   std::shared_ptr<IterativeVelocityController> icontroller);

  IterativeMotorVelocityController(const IterativeMotorVelocityControllerArgs &iparams);

  /**
   * Do one iteration of the controller.
   *
   * @param inewReading new measurement
   * @return controller output
   */
  virtual double step(const double ireading) override;

  /**
   * Sets the target for the controller.
   */
  virtual void setTarget(const double itarget) override;

  /**
   * Returns the last calculated output of the controller.
   */
  virtual double getOutput() const override;

  /**
   * Returns the last error of the controller.
   */
  virtual double getError() const override;

  /**
   * Returns the last derivative (change in error) of the controller.
   */
  virtual double getDerivative() const override;

  /**
   * Returns whether the controller has settled at the target. Determining what settling means is
   * implementation-dependent.
   *
   * @return whether the controller is settled
   */
  virtual bool isSettled() override;

  /**
   * Set time between loops in ms.
   *
   * @param isampleTime time between loops in ms
   */
  virtual void setSampleTime(const QTime isampleTime) override;

  /**
   * Set controller output bounds.
   *
   * @param imax max output
   * @param imin min output
   */
  virtual void setOutputLimits(double imax, double imin) override;

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

  /**
   * Get the last set sample time.
   *
   * @return sample time
   */
  virtual QTime getSampleTime() const override;

  protected:
  std::shared_ptr<AbstractMotor> motor;
  std::shared_ptr<IterativeVelocityController> controller;
};
} // namespace okapi

#endif
