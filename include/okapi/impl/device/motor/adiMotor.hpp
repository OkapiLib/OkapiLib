/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ADIMOTOR_HPP_
#define _OKAPI_ADIMOTOR_HPP_

#include "api.h"
#include "okapi/api/control/controllerOutput.hpp"

namespace okapi {
class ADIMotor : public ControllerOutput<double> {
  public:
  ADIMotor(std::uint8_t iport, bool ireverse = false);

  /**
   * Set the voltage to the motor.
   *
   * @param ivoltage voltage
   */
  virtual void moveVoltage(std::int32_t ivoltage) const;

  /**
   * Writes the value of the controller output. This method might be automatically called in another
   * thread by the controller. The range of input values is expected to be [-1, 1].
   *
   * @param ivalue the controller's output in the range [-1, 1]
   */
  virtual void controllerSet(double ivalue) override;

  protected:
  const pros::ADIMotor motor;
  const std::int8_t reversed;
};
} // namespace okapi

#endif
