/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_MOTOR_HPP_
#define _OKAPI_MOTOR_HPP_

#include "api.h"
#include "okapi/device/abstractMotor.hpp"

namespace okapi {
class Motor : public AbstractMotor {
  public:
  using AbstractMotor::AbstractMotor;

  virtual ~Motor();

  void controllerSet(const double ivalue) override;

  IntegratedEncoder getEncoder() const override;
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
