#ifndef OKAPI_QUADENCODER
#define OKAPI_QUADENCODER

#include <API.h>

namespace okapi {
  class QuadEncoder {
  public:
    QuadEncoder(const unsigned char iportTop, const unsigned char iportBottom):
      enc(encoderInit(iportTop, iportBottom, false)) {}

    QuadEncoder(const unsigned char iportTop, const unsigned char iportBottom, const bool ireversed):
      enc(encoderInit(iportTop, iportBottom, ireversed)) {}

    int get() const { return encoderGet(enc); }
    void reset() const { encoderReset(enc); }
  private:
    const Encoder enc;
  };
}

#endif /* end of include guard: OKAPI_QUADENCODER */
