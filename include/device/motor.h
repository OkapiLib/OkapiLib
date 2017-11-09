#ifndef OKAPI_MOTOR
#define OKAPI_MOTOR

#include "PAL/PAL.h"

namespace okapi {
  namespace motor {
    constexpr int trueSpeed[128] = {
      0,18,21,21,22,23,24,24,24,25,25,26,26,26,27,27,27,28,28,28,28,29,29,29,
      30,30,30,30,31,31,31,31,32,32,33,33,33,33,34,34,34,35,35,35,36,36,36,37,
      37,37,38,38,38,38,39,39,39,40,40,40,41,41,41,42,42,42,43,43,43,44,44,44,
      45,45,45,46,46,46,47,47,48,48,49,49,50,50,51,51,51,52,52,53,53,54,54,55,
      55,56,57,58,59,59,60,61,62,64,65,66,67,67,68,71,72,73,73,76,78,79,81,82,
      83,90,95,115,122,123,124,127
    };
    constexpr int cubicSpeed[128] = {
      0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,2,2,2,2,2,2,3,
      3,3,3,4,4,4,5,5,5,6,6,6,7,7,8,8,9,9,10,10,11,11,12,13,13,14,15,16,16,17,
      18,19,19,20,21,22,23,24,25,26,27,28,29,31,32,33,34,35,37,38,39,41,42,44,
      45,47,48,50,51,53,55,57,58,60,62,64,66,68,70,72,74,76,78,80,83,85,87,89,
      92,94,97,99,102,104,107,110,113,115,118,121,124,127
    };
  }

  class Motor {
  public:
    explicit constexpr Motor():
      port(0),
      sign(1) {}

    explicit constexpr Motor(const unsigned char iport, const int isign):
      port(iport),
      sign(isign) {}

    constexpr Motor(const Motor& other):
      port(other.port),
      sign(other.sign) {}

    virtual void set(const int val) const { PAL::motorSet(port, val * sign); }

    virtual void setTS(const int val) const {
      if (val > 127)
        PAL::motorSet(port, motor::trueSpeed[127] * sign);
      else if (val < -127)
        PAL::motorSet(port, motor::trueSpeed[127] * -1 * sign);
      else if (val < 0)
        PAL::motorSet(port, motor::trueSpeed[-1 * val] * -1 * sign);
      else
        PAL::motorSet(port, motor::trueSpeed[val] * sign);
    }

  protected:
    const unsigned char port;
    const int sign;
  };

  class CubicMotor : public Motor {
  public:
    explicit constexpr CubicMotor():
      Motor() {}

    explicit constexpr CubicMotor(const unsigned char iport, const int isign):
      Motor(iport, isign) {}
      
    constexpr CubicMotor(const CubicMotor& other):
      CubicMotor(other.port, other.sign) {}

    virtual void set(const int val) const override {
      if (val > 127)
        PAL::motorSet(port, motor::cubicSpeed[127] * sign);
      else if (val < -127)
        PAL::motorSet(port, motor::cubicSpeed[127] * -1 * sign);
      else if (val < 0)
        PAL::motorSet(port, motor::cubicSpeed[-1 * val] * -1 * sign);
      else
        PAL::motorSet(port, motor::cubicSpeed[val] * sign);
    }

    virtual void setTS(const int val) const override { Motor::setTS(val); }
  };

  class SlewMotor : public Motor {
  public:
    SlewMotor(const Motor& imotor, const float islewRate):
      Motor(imotor),
      slewRate(islewRate) {}

    virtual void set(const int val) {
      slew(val);
      Motor::set((int)artSpeed);
    }

    virtual void setTS(const int val) {
      slew(val);
      Motor::setTS((int)artSpeed);
    }
  protected:
    float slewRate, artSpeed = 0;

    __attribute__((always_inline))
    void slew(const int val) {
      if (artSpeed != val) {
        if (val > artSpeed) {
          artSpeed += slewRate;
          if (artSpeed > val)
            artSpeed = static_cast<float>(val);
        } else if (val < artSpeed) {
          artSpeed -= slewRate;
          if (artSpeed < val)
            artSpeed = static_cast<float>(val);
        }
      }
    }
  };

  class CubicSlewMotor : public CubicMotor {
  public:
    CubicSlewMotor(const CubicMotor& imotor, const float islewRate):
      CubicMotor(imotor),
      slewRate(islewRate) {}
    
    virtual void set(const int val) {
      slew(val);
      CubicMotor::set((int)artSpeed);
    }

    virtual void setTS(const int val) {
      slew(val);
      CubicMotor::setTS((int)artSpeed);
    }
  protected:
    float slewRate, artSpeed = 0;
    
    __attribute__((always_inline))
    void slew(const int val) {
      if (artSpeed != val) {
        if (val > artSpeed) {
          artSpeed += slewRate;
          if (artSpeed > val)
            artSpeed = static_cast<float>(val);
        } else if (val < artSpeed) {
          artSpeed -= slewRate;
          if (artSpeed < val)
            artSpeed = static_cast<float>(val);
        }
      }
    }
  };

  inline namespace literals {
    constexpr Motor operator"" _m(const unsigned long long int m) { return Motor(static_cast<unsigned char>(m), 1); }
    constexpr Motor operator"" _rm(const unsigned long long int m) { return Motor(static_cast<unsigned char>(m), -1); }
    constexpr CubicMotor operator"" _m3(const unsigned long long int m) { return CubicMotor(static_cast<unsigned char>(m), 1); }
    constexpr CubicMotor operator"" _rm3(const unsigned long long int m) { return CubicMotor(static_cast<unsigned char>(m), -1); }
  }
}

#endif /* end of include guard: OKAPI_MOTOR */
