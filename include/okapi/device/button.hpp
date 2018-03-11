/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
 #ifndef _OKAPI_BUTTON_HPP_
 #define _OKAPI_BUTTON_HPP_

#include "api.h"
#include <functional>

namespace okapi {
  class Button {
  public:
    Button(const uint8_t iport, const bool iinverted = false);
    Button(pros::Controller &icontroller, const controller_digital_e_t ibtn,
      const bool iinverted = false);
    virtual ~Button();

    /**
     * Return whether the button is current pressed.
     **/
    virtual bool isPressed() = 0;

    /**
     * Return whether there just was a rising or falling edge.
     **/
    virtual bool edge() = 0;

    /**
     * Return whether there was just a rising edge.
     **/
    virtual bool risingEdge() = 0;

    /**
     * Return whether there was just a falling edge.
     **/
    virtual bool fallingEdge() = 0;

  private:
    const bool isADIButton, isControllerButton;

    pros::ADIButton btn;
    uint8_t port;

    pros::Controller &controller;
    controller_digital_e_t controllerButton;

    const bool inverted;
    bool wasPressedLast;

    /**
     * Whether the button is currently pressed. Same result from isPressed() but isPressed()
     * updates wasPressedLast.
     **/
    bool currentlyPressed();
  };

  inline namespace literals {
    /**
     * Non-inverted button.
     **/
    Button operator"" _b(const unsigned long long int iport);

    /**
     * Inverted button.
     **/
    Button operator"" _ib(const unsigned long long int iport);
  }
}

 #endif
