#ifndef OKAPI_IME
#define OKAPI_IME

#include <API.h>
#include "device/rotarySensor.h"

namespace okapi {
  class IME : public RotarySensor {
  public:
    IME(const unsigned int iindex):
      index(iindex),
      reversed(1) {}

    IME(const unsigned int iindex, const bool ireversed):
        index(iindex),
        reversed(ireversed ? -1 : 1) {}

    int get() override { imeGet(index, &val); return reversed * val; }
    void reset() override { imeReset(index); }
  private:
    unsigned int index;
    const int reversed;
    int val;
  };
}

#endif /* end of include guard: OKAPI_IME */
