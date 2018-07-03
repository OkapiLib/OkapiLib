/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_VELMATHFACTORY_HPP_
#define _OKAPI_VELMATHFACTORY_HPP_

#include "okapi/api/filter/velMath.hpp"

namespace okapi {
class VelMathFactory {
  public:
  /**
   * Velocity math helper. Calculates filtered velocity. Throws a std::invalid_argument exception
   * if iticksPerRev is zero. Averages the last two readings.
   *
   * @param iticksPerRev number of ticks per revolution (or whatever units you are using)
   */
  static VelMath create(double iticksPerRev);

  /**
   * Velocity math helper. Calculates filtered velocity. Throws a std::invalid_argument exception
   * if iticksPerRev is zero.
   *
   * @param iticksPerRev number of ticks per revolution (or whatever units you are using)
   * @param ifilter filter used for filtering the calculated velocity
   */
  static VelMath create(double iticksPerRev, std::shared_ptr<Filter> ifilter);
};
} // namespace okapi

#endif
