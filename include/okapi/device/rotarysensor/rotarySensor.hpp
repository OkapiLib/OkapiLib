/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ROTARYSENSOR_HPP_
#define _OKAPI_ROTARYSENSOR_HPP_

#include "api.h"
#include "okapi/control/controllerInput.hpp"

namespace okapi {
class RotarySensor : public ControllerInput {
  public:
  virtual ~RotarySensor();

  /**
   * Get the current sensor value.
   *
   * @return the current sensor value, or ``PROS_ERR`` on a failure.
   */
  virtual std::int32_t get() const = 0;
};
} // namespace okapi

#endif
