/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_RATE_HPP_
#define _OKAPI_RATE_HPP_

#include "api.h"
#include "okapi/units/QFrequency.hpp"

namespace okapi {
class Rate {
  public:
  Rate();

  virtual ~Rate();

  /**
   * Delay the current task such that it runs at the given frequency. The first delay will
   * run for 1000/(ihz). Subsequent delays will adjust according to the previous runtime of the
   * task.
   *
   * @param ihz rate
   */
  virtual void delayHz(const QFrequency ihz);

  protected:
  std::uint32_t lastTime = 0;
};
} // namespace okapi

#endif
