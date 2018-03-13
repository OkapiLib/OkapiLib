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
  class AverageFilter : public Filter {
  public:
    AverageFilter();

    virtual ~AverageFilter();

    float filter(const float ireading) override;

    float getOutput() const override;

  private:
    std::array<float, n> data;
    float index, output;
  };
}

#endif
