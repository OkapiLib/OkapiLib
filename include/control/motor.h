#ifndef OKAPI_MOTOR
#define OKAPI_MOTOR

#include <API.h>

namespace okapi {
  class Motor {
    public:
      explicit constexpr Motor():
        port(0),
        sign(1) {}

      explicit constexpr Motor(const unsigned long long int iport, const int isign):
        port(iport),
        sign(isign) {}

      Motor& operator=(const Motor& m) {
        port = m.port;
        sign = m.sign;
        return *this;
      }

      void set(const int val) const { motorSet(port, val * sign); }

      void setTS(const int val) const { motorSet(port, trueSpeed[val] * sign); }

    private:
      unsigned long long int port;
      int sign;
      static constexpr int trueSpeed[128] = {
        0,  0,  0,  0,  0,  0,   0,   0,   0,  0,
        0,  21, 21, 21, 22, 22,  22,  23,  24, 24,
        25, 25, 25, 25, 26, 27,  27,  28,  28, 28,
        28, 29, 30, 30, 30, 31,  31,  32,  32, 32,
        33, 33, 34, 34, 35, 35,  35,  36,  36, 37,
        37, 37, 37, 38, 38, 39,  39,  39,  40, 40,
        41, 41, 42, 42, 43, 44,  44,  45,  45, 46,
        46, 47, 47, 48, 48, 49,  50,  50,  51, 52,
        52, 53, 54, 55, 56, 57,  57,  58,  59, 60,
        61, 62, 63, 64, 65, 66,  67,  67,  68, 70,
        71, 72, 72, 73, 74, 76,  77,  78,  79, 79,
        80, 81, 83, 84, 84, 86,  86,  87,  87, 88,
        88, 89, 89, 90, 90, 127, 127, 127 };
  };

  constexpr Motor operator"" _m(const unsigned long long int m) { return Motor(m, 1); }
  constexpr Motor operator"" _rm(const unsigned long long int m) { return Motor(m, -1); }  
}

#endif /* end of include guard: OKAPI_MOTOR */
