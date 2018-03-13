/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/filter/averageFilter.hpp"

namespace okapi {
template <std::size_t n> AverageFilter<n>::AverageFilter() : data(), index(0), output(0) {
}

template <std::size_t n> AverageFilter<n>::~AverageFilter() = default;

template <std::size_t n> float AverageFilter<n>::filter(const float ireading) {
  data[index++] = ireading;
  if (index > n)
    index = 0;

  output = 0.0;
  for (size_t i = 0; i < n; i++)
    output += data[i];
  output /= (float)n;

  return output;
}

template <std::size_t n> float AverageFilter<n>::getOutput() const {
  return output;
}
} // namespace okapi
