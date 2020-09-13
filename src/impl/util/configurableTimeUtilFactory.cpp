/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/util/configurableTimeUtilFactory.hpp"

namespace okapi {
ConfigurableTimeUtilFactory::ConfigurableTimeUtilFactory(const double iatTargetError,
                                                         const double iatTargetDerivative,
                                                         const QTime &iatTargetTime)
  : atTargetError(iatTargetError),
    atTargetDerivative(iatTargetDerivative),
    atTargetTime(iatTargetTime) {
}

TimeUtil ConfigurableTimeUtilFactory::create() {
  return withSettledUtilParams(atTargetError, atTargetDerivative, atTargetTime);
}
} // namespace okapi
