/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef OKAPI_AVERAGEFILTER_HPP_
#define OKAPI_AVERAGEFILTER_HPP_

#include <cstddef>
#include <array>
#include "okapi/filter/filter.hpp"

namespace okapi {
  template<std::size_t n>
  class AverageFilter final : public Filter {
  public:
    AverageFilter():
      data(),
      index(0),
      output(0) {}

    virtual ~AverageFilter() { delete &data; }

    float filter(const float ireading) override {
      data[index++] = ireading;
      if (index > n)
        index = 0;

      output = 0.0;
      for (size_t i = 0; i < n; i++)
        output += data[i];
      output /= (float)n;

      return output;
    }

    float getOutput() const override { return output; }

  private:
    std::array<float, n> data;
    float index, output;
  };
}

#endif
