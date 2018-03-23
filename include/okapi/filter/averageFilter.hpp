/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef OKAPI_AVERAGEFILTER_HPP_
#define OKAPI_AVERAGEFILTER_HPP_

#include "okapi/filter/filter.hpp"
#include <array>
#include <cstddef>

namespace okapi {
/**
 * @param n number of taps in the filter
 */
template <std::size_t n> class AverageFilter : public Filter {
  public:
  /**
   * Averaging filter.
   */
  AverageFilter() : data(), index(0), output(0) {
  }

  virtual ~AverageFilter() = default;

  double filter(const double ireading) override {
    data[index++] = ireading;
    if (index > n) {
      index = 0;
    }

    output = 0.0;
    for (size_t i = 0; i < n; i++)
      output += data[i];
    output /= (double)n;

    return output;
  }

  double getOutput() const override {
    return output;
  }

  private:
  std::array<double, n> data;
  double index, output;
};
} // namespace okapi

#endif
