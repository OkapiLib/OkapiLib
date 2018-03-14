/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_ABSTRACTMOTOR_HPP_
#define _OKAPI_ABSTRACTMOTOR_HPP_

#include "api.h"
#include "okapi/device/integratedEncoder.hpp"

namespace okapi {
class AbstractMotor {
  public:
  virtual int32_t moveAbsolute(const double position, const int32_t velocity) const = 0;

  virtual int32_t moveRelative(const double position, const int32_t velocity) const = 0;

  virtual int32_t moveVelocity(const int16_t velocity) const = 0;

  virtual int32_t moveVoltage(const int16_t voltage) const = 0;

  virtual IntegratedEncoder getEncoder() const = 0;
};
} // namespace okapi

#endif
