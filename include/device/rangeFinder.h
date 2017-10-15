#ifndef OKAPI_RANGEFINDER
#define OKAPI_RANGEFINDER

#include <API.h>

namespace okapi {
  class RangeFinder {
  public:
    RangeFinder(const unsigned char iportTop, const unsigned char iportBottom):
      ultra(ultrasonicInit(iportTop, iportBottom)) {}

    int get() const { return ultrasonicGet(ultra); }
  private:
    const Ultrasonic ultra;
  };
}

#endif /* end of include guard: OKAPI_RANGEFINDER */
