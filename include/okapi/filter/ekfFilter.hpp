/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_EKFFILTER_HPP_
#define _OKAPI_EKFFILTER_HPP_

#include "okapi/filter/filter.hpp"
#include "okapi/util/mathUtil.hpp"

namespace okapi {
class EKFFilter final : public Filter {
  public:
  EKFFilter(const double iQ = 0.0001, const double iR = ipow(0.2, 2));

  virtual ~EKFFilter();

  /**
   * Filters a reading. Assumes the control input is zero.
   *
   * @param ireading new measurement
   * @return filtered result
   */
  double filter(const double ireading) override;

  /**
   * Filters a reading with a control input.
   *
   * @param ireading new measurement
   * @param icontrol control input
   * @return filtered result
   */
  double filter(const double ireading, const double icontrol);

  /**
   * Returns the previous output from filter.
   *
   * @return the previous output from filter
   */
  double getOutput() const override;

  private:
  const double Q, R;
  double xHat, xHatPrev, xHatMinus, P, Pprev, Pminus, K;
};
} // namespace okapi

#endif
