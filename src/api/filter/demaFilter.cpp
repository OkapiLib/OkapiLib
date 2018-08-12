/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/filter/demaFilter.hpp"

namespace okapi {
DemaFilter::DemaFilter(const double ialpha, const double ibeta) : alpha(ialpha), beta(ibeta) {
}

double DemaFilter::filter(const double ireading) {
  outputS = (alpha * ireading) + ((1.0 - alpha) * (lastOutputS + lastOutputB));
  outputB = (beta * (outputS - lastOutputS)) + ((1.0 - beta) * lastOutputB);
  lastOutputS = outputS;
  lastOutputB = outputB;
  return outputS + outputB;
}

double DemaFilter::getOutput() const {
  return outputS + outputB;
}

void DemaFilter::setGains(const double ialpha, const double ibeta) {
  alpha = ialpha;
  beta = ibeta;
}
} // namespace okapi
