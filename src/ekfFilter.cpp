/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/filter/ekfFilter.hpp"

namespace okapi {
EKFFilter::EKFFilter(const double iQ, const double iR)
  : Q(iQ), R(iR), xHat(0), xHatPrev(0), xHatMinus(0), P(0), Pprev(1), Pminus(0), K(0) {
}

EKFFilter::~EKFFilter() = default;

/**
 * Filters a reading.
 *
 * @param ireading new measurement
 * @return filtered result
 */
double EKFFilter::filter(const double ireading) {
  return filter(ireading, 0);
}

/**
 * Filters a reading.
 *
 * @param ireading new measurement
 * @param icontrol control input
 * @return filtered result
 */
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

/**
 * Returns the previous output from filter.
 *
 * @return the previous output from filter
 */
double EKFFilter::getOutput() const {
  return xHat;
}
} // namespace okapi
