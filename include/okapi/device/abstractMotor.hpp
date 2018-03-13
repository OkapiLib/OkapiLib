/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_ABSTRACTMOTOR_HPP_
#define _OKAPI_ABSTRACTMOTOR_HPP_

#include "api.h"

namespace okapi {
class AbstractMotor {
  public:
  virtual int32_t set_velocity(const int16_t ivelocity) const = 0;
};
} // namespace okapi

#endif
