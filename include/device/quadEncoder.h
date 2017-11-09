#ifndef OKAPI_QUADENCODER
#define OKAPI_QUADENCODER

#include "device/rotarySensor.h"
#include "PAL/PAL.h"

namespace okapi {
  class QuadEncoder : public RotarySensor {
  public:
    QuadEncoder(const unsigned char iportTop, const unsigned char iportBottom):
      enc(encoderInit(iportTop, iportBottom, false)) {}

    QuadEncoder(const unsigned char iportTop, const unsigned char iportBottom, const bool ireversed):
      enc(encoderInit(iportTop, iportBottom, ireversed)) {}

    int get() override { return PAL::encoderGet(enc); }
    void reset() override { PAL::encoderReset(enc); }
  private:
    const Encoder enc;
  };
}

#endif /* end of include guard: OKAPI_QUADENCODER */
