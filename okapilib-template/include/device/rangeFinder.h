#ifndef OKAPI_RANGEFINDER
#define OKAPI_RANGEFINDER

#include <array>
#include "PAL/PAL.h"

namespace okapi {
  class RangeFinder {
  public:
    RangeFinder(const unsigned char iportTop, const unsigned char iportBottom):
      ultra(ultrasonicInit(iportTop, iportBottom)),
      vals{0},
      index(0) {}

    int get() {
      const int val = PAL::ultrasonicGet(ultra);
      vals[index++ % len] = val;
      return val;
    }

    int getFiltered() {
      const int val = PAL::ultrasonicGet(ultra);
      vals[index++ % len] = val;

      sort(0, 1, index);
      sort(3, 4, index);
      sort(0, 3, index);
      sort(1, 4, index);
      sort(1, 2, index);
      sort(2, 3, index);
      sort(1, 2, index);

      return vals[2];
    }
  private:
    const Ultrasonic ultra;
    static constexpr size_t len = 5;
    std::array<int, len> vals;
    size_t index;

    void sort(const size_t a, const size_t b, size_t& index) {
      if (vals[a] > vals[b]) {
        if (index % len == a)
          index = a;
        
        const int temp = vals[a];
        vals[a] = vals[b];
        vals[b] = temp;
      }
    }
  };
}

#endif /* end of include guard: OKAPI_RANGEFINDER */
