#ifndef OKAPI_EMAFILTER
#define OKAPI_EMAFILTER

#include "filter/filter.h"

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

#endif /* end of include guard: OKAPI_EMAFILTER */
