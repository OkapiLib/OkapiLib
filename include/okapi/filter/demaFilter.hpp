/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef OKAPI_DEMAFILTER_HPP_
#define OKAPI_DEMAFILTER_HPP_

#include "okapi/filter/filter.hpp"

namespace okapi {
  class DemaFilter : public Filter {
  public:
    /**
     * Double exponential moving average filter.
     * 
     * @param ialpha alpha gain
     * @param ibeta beta gain
     */
    DemaFilter(const double ialpha, const double ibeta);

    /**
     * Filters a reading.
     * 
     * @param reading new measurement
     * @return filtered result
     */
    double filter(const double ireading) override;

    /**
     * Returns the previous output from filter.
     * 
     * @return the previous output from filter
     */
    double getOutput() const override;

    /**
     * Set filter gains.
     * 
     * @param ialpha alpha gain
     * @param ibeta beta gain
     */
    void setGains(const double ialpha, const double ibeta);

  private:
    double alpha, beta;
    double outputS, lastOutputS;
    double outputB, lastOutputB;
  };
}

#endif
