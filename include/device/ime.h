#ifndef OKAPI_IME
#define OKAPI_IME

#include "device/rotarySensor.h"
#include "PAL/PAL.h"

namespace okapi {
  class IME : public RotarySensor {
  public:
    explicit constexpr IME(const unsigned char iindex):
      index(iindex),
      reversed(1),
      val(0) {}

    explicit constexpr IME(const unsigned char iindex, const bool ireversed):
        index(iindex),
        reversed(ireversed ? -1 : 1),
        val(0) {}

    int get() override { PAL::imeGet(index, &val); return reversed * val; }
    void reset() override { PAL::imeReset(index); }
  private:
    unsigned char index;
    const int reversed;
    int val;
  };
  
  inline namespace literals {
    constexpr IME operator"" _ime(const unsigned long long int p) { return IME(static_cast<unsigned char>(p), false); }
    constexpr IME operator"" _rime(const unsigned long long int p) { return IME(static_cast<unsigned char>(p), true); }
  }
}

#endif /* end of include guard: OKAPI_IME */
