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
#include <memory>

namespace okapi {
class VelMathFactory {
  public:
  /**
   * Velocity math helper. Calculates filtered velocity. Throws a std::invalid_argument exception
   * if iticksPerRev is zero. Averages the last two readings.
   *
   * @param iticksPerRev number of ticks per revolution (or whatever units you are using)
   */
  static VelMath create(double iticksPerRev, QTime isampleTime = 0_ms);

  static std::unique_ptr<VelMath> createPtr(double iticksPerRev, QTime isampleTime = 0_ms);

  /**
   * Velocity math helper. Calculates filtered velocity. Throws a std::invalid_argument exception
   * if iticksPerRev is zero.
   *
   * @param iticksPerRev number of ticks per revolution (or whatever units you are using)
   * @param ifilter filter used for filtering the calculated velocity
   */
  static VelMath
  create(double iticksPerRev, std::shared_ptr<Filter> ifilter, QTime isampleTime = 0_ms);

  static std::unique_ptr<VelMath>
  createPtr(double iticksPerRev, std::shared_ptr<Filter> ifilter, QTime isampleTime = 0_ms);

  static std::unique_ptr<VelMath> createPtr(const VelMathArgs &ivelMathArgs);
};
} // namespace okapi

#endif
