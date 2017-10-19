#ifndef OKAPI_POTENTIOMETER
#define OKAPI_POTENTIOMETER

#include <API.h>

namespace okapi {
  class Potentiometer {
  public:
    explicit constexpr Potentiometer():
      port(0),
      inverted(false) {}
      
    explicit constexpr Potentiometer(const unsigned long long int iport):
      port(iport),
      inverted(false) {}

    explicit constexpr Potentiometer(const unsigned long long int iport, const bool iinverted):
      port(iport),
      inverted(iinverted) {}

    int get() const { return inverted ? 4095 - analogRead(port) : analogRead(port); }
  private:
    const unsigned char port;
    const bool inverted;
  };

  inline namespace literals {
    constexpr Potentiometer operator"" _p(const unsigned long long int p) { return Potentiometer(static_cast<unsigned char>(p), false); }
    constexpr Potentiometer operator"" _ip(const unsigned long long int p) { return Potentiometer(static_cast<unsigned char>(p), true); }
  }
}

#endif /* end of include guard: OKAPI_POTENTIOMETER */