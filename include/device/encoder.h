#ifndef OKAPI_ENCODER
#define OKAPI_ENCODER

#include <API.h>

namespace okapi {
  class Encoder {
  public:
    explicit constexpr Encoder():
      enc(encoderInit(0, 1, false)) {}

    explicit constexpr Encoder(const unsigned char iportTop, const unsigned char iportBottom, const bool ireversed):
      enc(encoderInit(iportTop, iportBottom, ireversed)) {}

    int get() const { return encoderGet(enc); }
    void reset() { encoderReset(enc); }
  private:
    const Encoder enc;
  };

  constexpr Encoder operator"" _e(const unsigned long long int portTop, const unsigned long long int portBottom) { return Encoder(portTop, portBottom, false); }
  constexpr Encoder operator"" _re(const unsigned long long int portTop, const unsigned long long int portBottom) { return Encoder(portTop, portBottom, true); }
}

#endif /* end of include guard: OKAPI_ENCODER */
