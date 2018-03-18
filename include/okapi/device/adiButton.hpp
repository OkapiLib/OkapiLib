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

  /**
   * Return whether the button is current pressed.
   **/
  bool isPressed();

  /**
   * Return whether there just was a rising or falling edge.
   **/
  bool edge();

  /**
   * Return whether there was just a rising edge.
   **/
  bool risingEdge();

  /**
   * Return whether there was just a falling edge.
   **/
  bool fallingEdge();

  private:
  pros::ADIButton btn;
  uint8_t port;
  const bool inverted;
  bool wasPressedLast;

  bool currentlyPressed();
};
} // namespace okapi

#endif
