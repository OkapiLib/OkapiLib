/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_ROTARYSENSOR_HPP_
#define _OKAPI_ROTARYSENSOR_HPP_

#include "api.h"

namespace okapi {
class RotarySensor {
  public:
  virtual ~RotarySensor() = default;

  virtual int32_t get() const = 0;

  virtual int32_t reset() const = 0;
};
} // namespace okapi

#endif
