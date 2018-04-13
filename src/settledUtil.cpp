/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/control/util/settledUtil.hpp"
#include <cmath>

namespace okapi {
SettledUtil::SettledUtil(const double iatTargetError, const double iatTargetDerivative,
                         const uint32_t iatTargetTime)
  : atTargetError(iatTargetError),
    atTargetDerivative(iatTargetDerivative),
    atTargetTime(iatTargetTime) {
}

SettledUtil::SettledUtil(const uint32_t iatTargetTime) : atTargetTime(iatTargetTime) {
}

SettledUtil::SettledUtil(const double iatTargetError, const double iatTargetDerivative)
  : atTargetError(iatTargetError), atTargetDerivative(iatTargetDerivative) {
}

SettledUtil::~SettledUtil() = default;

bool SettledUtil::isSettled(const double ierror) {
  if (std::fabs(ierror) <= atTargetError) {
    atTargetTimer.placeHardMark();
  } else if (std::fabs(ierror - lastError) <= atTargetDerivative) {
    atTargetTimer.placeHardMark();
  } else {
    atTargetTimer.clearHardMark();
  }

  lastError = ierror;

  if (atTargetTimer.getDtFromHardMark() >= atTargetTime) {
    return true;
  }

  return false;
}
}
