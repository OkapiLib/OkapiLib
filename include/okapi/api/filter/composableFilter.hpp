/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/filter/filter.hpp"
#include <functional>
#include <initializer_list>
#include <memory>
#include <vector>

namespace okapi {
class ComposableFilter : public Filter {
  public:
  /**
   * A composable filter is a filter that consists of other filters. The input signal is passed
   * through each filter in sequence. The final output of this filter is the output of the last
   * filter.
   */
  ComposableFilter();

  /**
   * A composable filter is a filter that consists of other filters. The input signal is passed
   * through each filter in sequence. The final output of this filter is the output of the last
   * filter.
   *
   * @param ilist the filters to use in sequence
   */
  ComposableFilter(const std::initializer_list<std::shared_ptr<Filter>> &ilist);

  /**
   * Filters a value, like a sensor reading.
   *
   * @param ireading new measurement
   * @return filtered result
   */
  double filter(double ireading) override;

  /**
   * Returns the previous output from filter.
   *
   * @return the previous output from filter
   */
  double getOutput() const override;

  /**
   * Adds a filter to the end of the sequence.
   *
   * @param ifilter the filter to add
   */
  virtual void addFilter(const std::shared_ptr<Filter> &ifilter);

  protected:
  std::vector<std::shared_ptr<Filter>> filters;
  double output = 0;
};
} // namespace okapi
