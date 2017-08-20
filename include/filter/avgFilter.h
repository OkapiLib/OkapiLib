#ifndef OKAPI_AVGFILTER
#define OKAPI_AVGFILTER

#include <cstddef>
#include <array>
#include "filter/filter.h"
#include <API.h>

namespace okapi {
  template<std::size_t n>
  class AvgFilter : public Filter {
  public:
    AvgFilter():
      data(),
      index(0),
      output(0) {}

    virtual ~AvgFilter() { delete &data; }

    float filter(const float ireading) {
      data[index++] = ireading;
      if (index > n)
        index = 0;

      output = 0.0;
      for (size_t i = 0; i < n; i++)
        output += data[i];
      output /= (float)n;

      return output;
    }

    float getOutput() const { return output; }
  private:
    std::array<float, n> data;
    float index, output;
  };
}

#endif /* end of include guard: OKAPI_AVGFILTER */
