/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_TIMEUTILFACTORY_HPP_
#define _OKAPI_TIMEUTILFACTORY_HPP_

#include "okapi/api/util/timeUtil.hpp"

namespace okapi {
class TimeUtilFactory {
  public:
  static TimeUtil create();
};
} // namespace okapi

#endif