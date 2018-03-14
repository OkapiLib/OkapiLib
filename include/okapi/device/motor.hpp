/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_MOTOR_HPP_
#define _OKAPI_MOTOR_HPP_

#include "api.h"
#include "okapi/device/abstractMotor.hpp"

namespace okapi {
class Motor : public AbstractMotor {
  public:
  Motor(const uint8_t port, const bool reverse = false,
        const motor_encoder_units_e_t encoder_units = E_MOTOR_ENCODER_DEGREES,
        const motor_gearset_e_t gearset = E_MOTOR_GEARSET_36);

  int32_t moveAbsolute(const double position, const int32_t velocity) const override;

  int32_t moveRelative(const double position, const int32_t velocity) const override;

  int32_t moveVelocity(const int16_t velocity) const override;

  int32_t moveVoltage(const int16_t voltage) const override;

  IntegratedEncoder getEncoder() const override;

  private:
  pros::Motor motor;
};

inline namespace literals {
/**
 * Non-reversed motor.
 **/
okapi::Motor operator"" _m(const unsigned long long iport);

/**
 * Reversed motor.
 **/
okapi::Motor operator"" _rm(const unsigned long long iport);
} // namespace literals
} // namespace okapi

#endif
