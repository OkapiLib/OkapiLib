/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_MOTORCONTROLLER_HPP_
#define _OKAPI_MOTORCONTROLLER_HPP_

#include "api.h"
#include "okapi/control/iterative/iterativeVelocityController.hpp"
#include "okapi/device/motor/abstractMotor.hpp"
#include <array>
#include <memory>

namespace okapi {
class MotorController : public IterativeVelocityController {
  public:
  MotorController(const AbstractMotor &imotor, IterativeVelocityController &icontroller);

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
   * Set time between loops in ms.
   *
   * @param isampleTime time between loops in ms
   */
  virtual void setSampleTime(const uint32_t isampleTime) override;

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
   * Change whether the controll is off or on.
   */
  virtual void flipDisable() override;

  /**
   * Get the last set sample time.
   *
   * @return sample time
   */
  virtual uint32_t getSampleTime() const override;

  protected:
  const AbstractMotor &motor;
  IterativeVelocityController &controller;
};
} // namespace okapi

#endif
