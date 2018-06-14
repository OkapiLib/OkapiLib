/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/filter/ekfFilter.hpp"

namespace okapi {
EKFFilter::EKFFilter(const double iQ, const double iR) : Q(iQ), R(iR) {
}

double EKFFilter::filter(const double ireading) {
  return filter(ireading, 0);
}

double EKFFilter::filter(const double ireading, const double icontrol) {
  // Time update
  xHatMinus = xHatPrev + icontrol;
  Pminus = Pprev + Q;

  // Measurement update
  K = Pminus / (Pminus + R);
  xHat = xHatMinus + K * (ireading - xHatMinus);
  P = (1 - K) * Pminus;

  xHatPrev = xHat;
  Pprev = P;

  return xHat;
}

double EKFFilter::getOutput() const {
  return xHat;
}
} // namespace okapi
