/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef OKAPI_DEMAFILTER_HPP_
#define OKAPI_DEMAFILTER_HPP_

#include "okapi/filter/filter.hpp"
#include <ratio>

namespace okapi {
class DemaFilterArgs : public FilterArgs {
  public:
  DemaFilterArgs(const double ialpha, const double ibeta);

  const double alpha;
  const double beta;
};

class DemaFilter : public Filter {
  public:
  /**
   * Double exponential moving average filter.
   *
   * @param ialpha alpha gain
   * @param ibeta beta gain
   */
  DemaFilter(const double ialpha, const double ibeta);

  DemaFilter(const DemaFilterArgs &iargs);

  /**
   * Filters a value, like a sensor reading.
   *
   * @param reading new measurement
   * @return filtered result
   */
  virtual double filter(const double ireading) override;

  /**
   * Returns the previous output from filter.
   *
   * @return the previous output from filter
   */
  virtual double getOutput() const override;

  /**
   * Set filter gains.
   *
   * @param ialpha alpha gain
   * @param ibeta beta gain
   */
  virtual void setGains(const double ialpha, const double ibeta);

  protected:
  double alpha, beta;
  double outputS = 0;
  double lastOutputS = 0;
  double outputB = 0;
  double lastOutputB = 0;
};
} // namespace okapi

#endif
