/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
 #include "okapi/device/button.hpp"

namespace okapi {
  Button::Button(const uint8_t iport, const bool iinverted):
    isADIButton(true),
    isControllerButton(false),
    btn(iport),
    port(iport),
    inverted(iinverted),
    wasPressedLast(false) {}
  
  Button::Button(pros::Controller &icontroller, const controller_digital_e_t ibtn,
    const bool iinverted):
    isADIButton(false),
    isControllerButton(true),
    controller(icontroller),
    controllerButton(ibtn),
    inverted(iinverted),
    wasPressedLast(false) {}

  Button::~Button() {}

  bool Button::isPressed() {
    wasPressedLast = currentlyPressed();
    return wasPressedLast;
  }

  bool Button::edge() {
    const bool pressed = currentlyPressed();
    const bool out = pressed ^ wasPressedLast;
    wasPressedLast = pressed;
    return out;
  }

  bool Button::risingEdge() {
    return edge() && wasPressedLast;
  }

  bool Button::fallingEdge() {
    return edge() && !wasPressedLast;
  }

  bool Button::currentlyPressed() {
    if (isADIButton) {
      const bool pressed = btn.value_get() != 0;
      return inverted ? !pressed : pressed;
    } else if (isControllerButton) {
      const bool pressed = controller.get_digital(controllerButton);
      return inverted ? !pressed : pressed;
    } else {
      return false;
    }
  }

  inline namespace literals {
    Button operator"" _b(const unsigned long long int iport) {
      return Button(static_cast<uint8_t>(iport), false);
    }

    Button operator"" _ib(const unsigned long long int iport) {
      return Button(static_cast<uint8_t>(iport), true);
    }
  }
}
