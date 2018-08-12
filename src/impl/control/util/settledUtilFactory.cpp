/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/control/util/settledUtilFactory.hpp"
#include "okapi/impl/util/timer.hpp"

namespace okapi {
SettledUtil SettledUtilFactory::create(const double iatTargetError,
                                       const double iatTargetDerivative,
                                       const QTime iatTargetTime) {
  return SettledUtil(std::make_unique<Timer>(), iatTargetError, iatTargetDerivative, iatTargetTime);
}

std::unique_ptr<SettledUtil> SettledUtilFactory::createPtr(const double iatTargetError,
                                                           const double iatTargetDerivative,
                                                           const QTime iatTargetTime) {
  return std::make_unique<SettledUtil>(
    std::make_unique<Timer>(), iatTargetError, iatTargetDerivative, iatTargetTime);
}
} // namespace okapi
