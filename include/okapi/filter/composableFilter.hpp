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
#include <initializer_list>
#include <vector>

namespace okapi {
class ComposableFilter : public Filter {
  public:
  ComposableFilter(std::initalizer_list<Filter &> list) {
  }

  /**
   * Filters a value, like a sensor reading.
   *
   * @param ireading new measurement
   * @return filtered result
   */
  virtual double filter(const double ireading) {
    for (auto filt : data) {
      filt.filter(ireading);
    }
    return data.back().getOutput();
  }

  /**
   * Returns the previous output from filter.
   *
   * @return the previous output from filter
   */
  virtual double getOutput() const {
    return output;
  }

  protected:
  std::vector<Filter &> data;
  double output = 0;
};
}

#endif
