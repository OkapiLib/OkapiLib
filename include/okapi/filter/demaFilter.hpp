/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef OKAPI_DEMAFILTER_HPP_
#define OKAPI_DEMAFILTER_HPP_

#include "okapi/filter/filter.hpp"

namespace okapi {
  class DemaFilter final : public Filter {
  public:
    DemaFilter(const float ialpha, const float ibeta):
      alpha(ialpha),
      beta(ibeta),
      outputS(0),
      lastOutputS(0),
      outputB(0),
      lastOutputB(0) {}

    virtual ~DemaFilter() = default;

    float filter(const float ireading) override {
      outputS = (alpha * ireading) + ((1.0 - alpha) * (lastOutputS + lastOutputB));
      outputB = (beta * (outputS - lastOutputS)) + ((1.0 - beta) * lastOutputB);
      lastOutputS = outputS;
      lastOutputB = outputB;
      return outputS + outputB;
    }

    void setGains(const float ialpha, const float ibeta) {
      alpha = ialpha;
      beta = ibeta;
    }

    float getOutput() const override { return outputS + outputB; }

  private:
    float alpha, beta;
    float outputS, lastOutputS;
    float outputB, lastOutputB;
  };
}

#endif
