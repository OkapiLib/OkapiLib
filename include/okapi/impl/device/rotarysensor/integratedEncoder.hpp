/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "api.h"
#include "okapi/api/device/rotarysensor/continuousRotarySensor.hpp"
#include "okapi/impl/device/motor/motor.hpp"

namespace okapi {
class IntegratedEncoder : public ContinuousRotarySensor {
  public:
  /**
   * Integrated motor encoder. Uses the encoder inside the V5 motor.
   *
   * @param imotor the motor to use the encoder from.
   */
  explicit IntegratedEncoder(const pros::Motor &imotor);

  /**
   * Integrated motor encoder. Uses the encoder inside the V5 motor.
   *
   * @param imotor the motor to use the encoder from.
   */
  explicit IntegratedEncoder(const okapi::Motor &imotor);

  /**
   * Get the current sensor value.
   *
   * @return the current sensor value, or ``PROS_ERR`` on a failure.
   */
  virtual double get() const override;

  /**
   * Reset the sensor to zero.
   *
   * @return 1 on success, PROS_ERR on fail
   */
  virtual std::int32_t reset() override;

  /**
   * Get the sensor value for use in a control loop. This method might be automatically called in
   * another thread by the controller.
   *
   * @return the current sensor value, or ``PROS_ERR`` on a failure.
   */
  virtual double controllerGet() override;

  protected:
  pros::Motor motor;
};
} // namespace okapi
