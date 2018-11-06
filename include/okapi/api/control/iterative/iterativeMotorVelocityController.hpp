/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/control/iterative/iterativeVelocityController.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include <array>
#include <memory>

namespace okapi {
class IterativeMotorVelocityController : public IterativeVelocityController<double, double> {
  public:
  /**
   * Velocity controller that automatically writes to the motor.
   */
  IterativeMotorVelocityController(
    const std::shared_ptr<AbstractMotor> &imotor,
    const std::shared_ptr<IterativeVelocityController<double, double>> &icontroller);

  /**
   * Do one iteration of the controller.
   *
   * @param inewReading new measurement
   * @return controller output
   */
  double step(double ireading) override;

  /**
   * Sets the target for the controller.
   */
  void setTarget(double itarget) override;

  /**
   * Writes the value of the controller output. This method might be automatically called in another
   * thread by the controller. The range of input values is expected to be [-1, 1].
   *
   * @param ivalue the controller's output in the range [-1, 1]
   */
  void controllerSet(double ivalue) override;

  /**
   * Gets the last set target, or the default target if none was set.
   *
   * @return the last target
   */
  double getTarget() override;

  /**
   * Returns the last calculated output of the controller.
   */
  double getOutput() const override;

  /**
   * Get the upper output bound.
   *
   * @return  the upper output bound
   */
  double getMaxOutput() override;

  /**
   * Get the lower output bound.
   *
   * @return the lower output bound
   */
  double getMinOutput() override;

  /**
   * Returns the last error of the controller.
   */
  double getError() const override;

  /**
   * Returns whether the controller has settled at the target. Determining what settling means is
   * implementation-dependent.
   *
   * @return whether the controller is settled
   */
  bool isSettled() override;

  /**
   * Set time between loops in ms.
   *
   * @param isampleTime time between loops in ms
   */
  void setSampleTime(QTime isampleTime) override;

  /**
   * Set controller output bounds.
   *
   * @param imax max output
   * @param imin min output
   */
  void setOutputLimits(double imax, double imin) override;

  /**
   * Resets the controller's internal state so it is similar to when it was first initialized, while
   * keeping any user-configured information.
   */
  void reset() override;

  /**
   * Changes whether the controller is off or on. Turning the controller on after it was off will
   * cause the controller to move to its last set target, unless it was reset in that time.
   */
  void flipDisable() override;

  /**
   * Sets whether the controller is off or on. Turning the controller on after it was off will
   * cause the controller to move to its last set target, unless it was reset in that time.
   *
   * @param iisDisabled whether the controller is disabled
   */
  void flipDisable(bool iisDisabled) override;

  /**
   * Returns whether the controller is currently disabled.
   *
   * @return whether the controller is currently disabled
   */
  bool isDisabled() const override;

  /**
   * Get the last set sample time.
   *
   * @return sample time
   */
  QTime getSampleTime() const override;

  protected:
  std::shared_ptr<AbstractMotor> motor;
  std::shared_ptr<IterativeVelocityController<double, double>> controller;
};
} // namespace okapi
