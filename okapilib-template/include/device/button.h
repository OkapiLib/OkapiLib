#ifndef OKAPI_BUTTON
#define OKAPI_BUTTON

#include <API.h>

namespace okapi {
  class Button {
  public:
    explicit constexpr Button():
      joystick(1),
      buttonGroup(8),
      port(0),
      inverted(false),
      isJoystick(false),
      wasPressedLast(false) {}
      
    explicit constexpr Button(const unsigned long long int iport, const bool iinverted = false):
      joystick(1),
      buttonGroup(8),
      port(iport),
      inverted(iinverted),
      isJoystick(false),
      wasPressedLast(iinverted) {}

    explicit constexpr Button(const unsigned char ijoystick, const unsigned char ibuttonGroup, const unsigned char ibutton, const bool iinverted = false):
      joystick(ijoystick),
      buttonGroup(ibuttonGroup),
      port(ibutton),
      inverted(iinverted),
      isJoystick(true),
      wasPressedLast(iinverted) {}


    bool isPressed() const {
      if (isJoystick)
        return inverted ? !joystickGetDigital(joystick, buttonGroup, port) : joystickGetDigital(joystick, buttonGroup, port);
      else
        return inverted ? !digitalRead(port) : digitalRead(port);
    }

    bool edge() {
      const bool pressed = isPressed();
      const bool out = pressed ^ wasPressedLast;
      wasPressedLast = pressed;
      return out;
    }
  private:
    const unsigned char joystick, buttonGroup, port;
    const bool inverted, isJoystick;
    bool wasPressedLast;
  };

  inline namespace literals {
    constexpr Button operator"" _b(const unsigned long long int p) { return Button(static_cast<unsigned char>(p), false); }
    constexpr Button operator"" _ib(const unsigned long long int p) { return Button(static_cast<unsigned char>(p), true); }
  }
}

#endif /* end of include guard: OKAPI_BUTTON */
