/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/util/timeUtilFactory.hpp"
#include "okapi/impl/control/util/settledUtilFactory.hpp"
#include "okapi/impl/util/rate.hpp"
#include "okapi/impl/util/timer.hpp"

namespace okapi {
TimeUtil TimeUtilFactory::create() {
  return TimeUtil(
    Supplier<std::unique_ptr<AbstractTimer>>([]() { return std::make_unique<Timer>(); }),
    Supplier<std::unique_ptr<AbstractRate>>([]() { return std::make_unique<Rate>(); }),
    Supplier<std::unique_ptr<SettledUtil>>([]() { return SettledUtilFactory::createPtr(); }));
}

TimeUtil TimeUtilFactory::withSettledUtilParams(const double iatTargetError,
                                                const double iatTargetDerivative,
                                                QTime iatTargetTime) {
  return TimeUtil(
    Supplier<std::unique_ptr<AbstractTimer>>([]() { return std::make_unique<Timer>(); }),
    Supplier<std::unique_ptr<AbstractRate>>([]() { return std::make_unique<Rate>(); }),
    Supplier<std::unique_ptr<SettledUtil>>([=]() {
      return SettledUtilFactory::createPtr(iatTargetError, iatTargetDerivative, iatTargetTime);
    }));
}
} // namespace okapi
