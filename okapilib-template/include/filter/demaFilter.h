#ifndef OKAPI_DEMAFILTER
#define OKAPI_DEMAFILTER

#include "filter/filter.h"

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

#endif /* end of include guard: OKAPI_DEMAFILTER */
