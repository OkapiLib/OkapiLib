/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/filter/composableFilter.hpp"

namespace okapi {
ComposableFilter::ComposableFilter() {
}

ComposableFilter::ComposableFilter(const std::initializer_list<std::function<Filter *()>> &list) {
  for (auto elem : list) {
    filters.push_back(elem());
  }
}

ComposableFilter::~ComposableFilter() {
  // Delete every filter that was allocated from the lambdas
  for (auto &elem : filters) {
    delete elem;
  }
}

double ComposableFilter::filter(const double ireading) {
  for (auto filt : filters) {
    filt->filter(ireading);
  }
  return filters.back()->getOutput();
}

double ComposableFilter::getOutput() const {
  return output;
}

void ComposableFilter::addFilter(const std::function<Filter *()> &filterAllocator) {
  filters.push_back(filterAllocator());
}
}
