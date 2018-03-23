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
#include "okapi/device/abstractMotor.hpp"
#include <array>
#include <memory>

namespace okapi {
class MotorController : public IterativeVelocityController {
  public:
  MotorController(const AbstractMotor &imotor, IterativeController &iptr);

  /**
   * Do one iteration of the controller.
   *
   * @param inewReading new measurement
   * @return controller output
   */
  virtual double step(const double ireading);

  /**
   * Sets the target for the controller.
   */
  void setTarget(const double itarget);

  /**
   * Returns the last calculated output of the controller.
   */
  double getOutput() const;

  /**
   * Returns the last error of the controller.
   */
  double getError() const;

  /**
   * Returns the last derivative (change in error) of the controller.
   */
  double getDerivative() const;

  /**
   * Set time between loops in ms.
   *
   * @param isampleTime time between loops in ms
   */
  void setSampleTime(const int isampleTime);

  /**
   * Set controller output bounds.
   *
   * @param imax max output
   * @param imin min output
   */
  void setOutputLimits(double imax, double imin);

  /**
   * Resets the controller so it can start from 0 again properly. Keeps configuration from
   * before.
   */
  void reset();

  /**
   * Change whether the controll is off or on.
   */
  void flipDisable();

  /**
   * Get the last set sample time.
   *
   * @return sample time
   */
  virtual uint32_t getSampleTime() const;

  protected:
  const AbstractMotor &motor;
  IterativeController &controller;
};
} // namespace okapi

#endif
