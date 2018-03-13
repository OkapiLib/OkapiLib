/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_ADIBUTTON_HPP_
#define _OKAPI_ADIBUTTON_HPP_

#include "okapi/device/button.hpp"

namespace okapi {
  class ADIButton : public Button {
  public:
    ADIButton(const uint8_t iport, const bool iinverted = false);

    virtual ~ADIButton();

    bool isPressed();

    bool edge();

    bool risingEdge();

    bool fallingEdge();

  private:
    pros::ADIButton btn;
    uint8_t port;
    const bool inverted;
    bool wasPressedLast;

    bool currentlyPressed();
  };
}

#endif
