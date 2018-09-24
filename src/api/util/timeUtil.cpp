/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/util/timeUtil.hpp"

namespace okapi {
TimeUtil::TimeUtil(const okapi::Supplier<std::unique_ptr<okapi::AbstractTimer>> &itimerSupplier,
                   const okapi::Supplier<std::unique_ptr<okapi::AbstractRate>> &irateSupplier,
                   const okapi::Supplier<std::unique_ptr<okapi::SettledUtil>> &isettledUtilSupplier)
  : timerSupplier(itimerSupplier),
    rateSupplier(irateSupplier),
    settledUtilSupplier(isettledUtilSupplier) {
}

std::unique_ptr<AbstractTimer> TimeUtil::getTimer() const {
  return timerSupplier.get();
}

std::unique_ptr<AbstractRate> TimeUtil::getRate() const {
  return rateSupplier.get();
}

std::unique_ptr<SettledUtil> TimeUtil::getSettledUtil() const {
  return settledUtilSupplier.get();
}

const Supplier<std::unique_ptr<AbstractTimer>> TimeUtil::getTimerSupplier() const {
  return timerSupplier;
}

const Supplier<std::unique_ptr<AbstractRate>> TimeUtil::getRateSupplier() const {
  return rateSupplier;
}

const Supplier<std::unique_ptr<SettledUtil>> TimeUtil::getSettledUtilSupplier() const {
  return settledUtilSupplier;
}
} // namespace okapi
