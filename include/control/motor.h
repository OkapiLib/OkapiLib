#ifndef OKAPI_MOTOR
#define OKAPI_MOTOR

#include <API.h>
#include <cmath>

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

      void setTS(const int val) const { using namespace std; motorSet(port, trueSpeed[abs((int)val)] * copysign(1, val) * sign); }

    private:
      unsigned long long int port;
      int sign;
      static constexpr int trueSpeed[128] = {
        0,18,21,21,22,23,24,24,24,25,25,26,26,26,27,27,27,28,28,28,28,29,29,29,
        30,30,30,30,31,31,31,31,32,32,33,33,33,33,34,34,34,35,35,35,36,36,36,37,
        37,37,38,38,38,38,39,39,39,40,40,40,41,41,41,42,42,42,43,43,43,44,44,44,
        45,45,45,46,46,46,47,47,48,48,49,49,50,50,51,51,51,52,52,53,53,54,54,55,
        55,56,57,58,59,59,60,61,62,64,65,66,67,67,68,71,72,73,73,76,78,79,81,82,
        83,90,95,115,122,123,124,127
      };
  };

  constexpr Motor operator"" _m(const unsigned long long int m) { return Motor(m, 1); }
  constexpr Motor operator"" _rm(const unsigned long long int m) { return Motor(m, -1); }
}

#endif /* end of include guard: OKAPI_MOTOR */
