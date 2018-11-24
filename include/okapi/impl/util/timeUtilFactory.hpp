/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/util/timeUtil.hpp"

namespace okapi {
class TimeUtilFactory {
  public:
  /**
   * Creates a default TimeUtil.
   */
  static TimeUtil create();

  /**
   * Creates a TimeUtil with custom SettledUtil params. See SettledUtil docs.
   */
  static TimeUtil withSettledUtilParams(double iatTargetError = 50,
                                        double iatTargetDerivative = 5,
                                        QTime iatTargetTime = 250_ms);
};
} // namespace okapi
