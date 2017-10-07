#ifndef OKAPI_BUTTON
#define OKAPI_BUTTON

namespace okapi {
  class Button {
  public:
    explicit constexpr Button():
      port(0),
      inverted(false) {}

    explicit constexpr Button()(const unsigned long long int iport, const bool iinverted):
      port(iport),
      inverted(iinverted) {}

    bool isPressed() const { return inverted ? !digitalRead(port) : digitalRead(port); }

  private:
    const unsigned char port;
    const bool inverted;
  };

  constexpr Button operator"" _b(const unsigned long long int b) { return Button(static_cast<unsigned char>(b), false); }
  constexpr Button operator"" _ib(const unsigned long long int b) { return Button(static_cast<unsigned char>(b), true); }
}

#endif /* end of include guard: OKAPI_BUTTON */
