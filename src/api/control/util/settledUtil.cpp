/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/util/settledUtil.hpp"
#include <cmath>

namespace okapi {
SettledUtil::SettledUtil(std::unique_ptr<AbstractTimer> iatTargetTimer,
                         const double iatTargetError,
                         const double iatTargetDerivative,
                         const QTime iatTargetTime)
  : atTargetError(iatTargetError),
    atTargetDerivative(iatTargetDerivative),
    atTargetTime(iatTargetTime),
    atTargetTimer(std::move(iatTargetTimer)) {
}

SettledUtil::~SettledUtil() = default;

bool SettledUtil::isSettled(const double ierror) {
  if (std::fabs(ierror) <= atTargetError && std::fabs(ierror - lastError) <= atTargetDerivative) {
    /**
     * Timer::getDtFromhardMark() returns 0_ms if there is no hard mark set, so this needs to be
     * special-cased. Setting atTargetTime to 0_ms means that the user wants to exit immediately
     * when in range of the target.
     */
    if (atTargetTime == 0_ms) {
      return true;
    }

    atTargetTimer->placeHardMark();
  } else {
    atTargetTimer->clearHardMark();
  }

  lastError = ierror;

  return atTargetTimer->getDtFromHardMark() > atTargetTime;
}

void SettledUtil::reset() {
  atTargetTimer->clearHardMark();
  lastError = 0;
}
} // namespace okapi
