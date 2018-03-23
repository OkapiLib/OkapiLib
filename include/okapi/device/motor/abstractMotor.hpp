/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ABSTRACTMOTOR_HPP_
#define _OKAPI_ABSTRACTMOTOR_HPP_

#include "api.h"
#include "okapi/control/controllerOutput.hpp"
#include "okapi/device/rotarysensor/integratedEncoder.hpp"

namespace okapi {
class AbstractMotor : public pros::Motor, public ControllerOutput {
  public:
  AbstractMotor(const uint8_t port, const bool reverse = false,
                const motor_encoder_units_e_t encoder_units = E_MOTOR_ENCODER_DEGREES,
                const motor_gearset_e_t gearset = E_MOTOR_GEARSET_36);

  virtual ~AbstractMotor();

  /**
   * Get the encoder associated with this motor.
   *
   * @return encoder for this motor
   */
  virtual IntegratedEncoder getEncoder() const = 0;
};
} // namespace okapi

#endif
