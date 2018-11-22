/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/filter/velMathFactory.hpp"
#include "okapi/api/filter/averageFilter.hpp"
#include "okapi/impl/util/timer.hpp"

namespace okapi {
VelMath VelMathFactory::create(const double iticksPerRev, const QTime isampleTime) {
  return VelMath(
    iticksPerRev, std::make_unique<AverageFilter<2>>(), isampleTime, std::make_unique<Timer>());
}

std::unique_ptr<VelMath> VelMathFactory::createPtr(const double iticksPerRev,
                                                   const QTime isampleTime) {
  return std::make_unique<VelMath>(
    iticksPerRev, std::make_unique<AverageFilter<2>>(), isampleTime, std::make_unique<Timer>());
}

VelMath VelMathFactory::create(const double iticksPerRev,
                               std::unique_ptr<Filter> ifilter,
                               const QTime isampleTime) {
  return VelMath(iticksPerRev, std::move(ifilter), isampleTime, std::make_unique<Timer>());
}

std::unique_ptr<VelMath> VelMathFactory::createPtr(const double iticksPerRev,
                                                   std::unique_ptr<Filter> ifilter,
                                                   const QTime isampleTime) {
  return std::make_unique<VelMath>(
    iticksPerRev, std::move(ifilter), isampleTime, std::make_unique<Timer>());
}
} // namespace okapi
