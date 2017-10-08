#ifndef OKAPI_ULTRASONIC
#define OKAPI_ULTRASONIC

#include <API.h>

namespace okapi {
  class Ultrasonic {
  public:
    explicit constexpr Ultrasonic():
      ultra(ultrasonicInit(0, 1)) {}

    explicit constexpr Ultrasonic(const unsigned char iportTop, const unsigned char iportBottom):
      ultra(ultrasonicInit(iportTop, iportBottom)) {}

    int get() const {
      return ultrasonicGet(ultra);
    }
  private:
    const Ultrasonic ultra;
  };

  constexpr Encoder operator"" _u(const unsigned long long int portTop, const unsigned long long int portBottom) { return Ultrasonic(portTop, portBottom); }
}

#endif /* end of include guard: OKAPI_ULTRASONIC */
