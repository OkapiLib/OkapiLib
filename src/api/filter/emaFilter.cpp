/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/filter/emaFilter.hpp"

namespace okapi {
EmaFilter::EmaFilter(const double ialpha) : alpha(ialpha) {
}

double EmaFilter::filter(const double ireading) {
  output = alpha * ireading + (1.0 - alpha) * lastOutput;
  lastOutput = output;
  return output;
}

double EmaFilter::getOutput() const {
  return output;
}

void EmaFilter::setGains(const double ialpha) {
  alpha = ialpha;
}
} // namespace okapi
