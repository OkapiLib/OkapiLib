/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_COMPOSABLEFILTER_HPP_
#define _OKAPI_COMPOSABLEFILTER_HPP_

#include "okapi/filter/filter.hpp"
#include <functional>
#include <initializer_list>
#include <vector>

namespace okapi {
class ComposableFilter : public Filter {
  public:
  /**
   * A composable filter is a filter that consists of other filters. The input signal is passed
   * through each filter in sequence. The output of the last filter is the output of this filter.
   */
  ComposableFilter();

  /**
   * A composable filter is a filter that consists of other filters. The input signal is passed
   * through each filter in sequence. The output of the last filter is the output of this filter.
   *
   * @param list the lambdas used to allocate filters
   */
  ComposableFilter(const std::initializer_list<std::function<Filter *()>> &list);

  virtual ~ComposableFilter();

  /**
   * Filters a value, like a sensor reading.
   *
   * @param ireading new measurement
   * @return filtered result
   */
  virtual double filter(const double ireading);

  /**
   * Returns the previous output from filter.
   *
   * @return the previous output from filter
   */
  virtual double getOutput() const;

  /**
   * Add a filter to the end of the sequence.
   *
   * @param filterAllocator a lambda called once to allocate a new filter
   */
  void addFilter(const std::function<Filter *()> &filterAllocator);

  protected:
  std::vector<Filter *> filters;
  double output = 0;
};
}

#endif
