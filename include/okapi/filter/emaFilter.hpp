/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_EMAFILTER_HPP_
#define _OKAPI_EMAFILTER_HPP_

#include "okapi/filter/filter.hpp"

namespace okapi {
  class EmaFilter final : public Filter {
  public:
    EmaFilter(const float ialpha, const float ibeta):
      alpha(ialpha),
      beta(ibeta),
      output(0),
      lastOutput(0) {}

    virtual ~EmaFilter() = default;

    float filter(const float ireading) override {
      output = alpha * ireading + (1.0 - alpha) * lastOutput;
      lastOutput = output;
      return output;
    }

    void setGains(const float ialpha, const float ibeta) {
      alpha = ialpha;
      beta = ibeta;
    }

    float getOutput() const override { return output; }

  private:
    float alpha, beta;
    float output, lastOutput;
  };
}

#endif
