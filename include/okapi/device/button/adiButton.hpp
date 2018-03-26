/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ADIBUTTON_HPP_
#define _OKAPI_ADIBUTTON_HPP_

#include "okapi/device/button/button.hpp"

namespace okapi {
class ADIButton : public Button {
  public:
  ADIButton(const uint8_t iport, const bool iinverted = false);

  /**
   * Return whether the button is current pressed.
   **/
  virtual bool isPressed();

  /**
   * Return whether the state of the button changed since the last time this method was
   * called.
   **/
  virtual bool changed();

  /**
   * Return whether the state of the button changed to being pressed since the last time this method
   * was called.
   **/
  virtual bool changedToPressed();

  /**
   * Return whether the state of the button to being not pressed changed since the last time this
   * method was called.
   **/
  virtual bool changedToReleased();

  protected:
  pros::ADIButton btn;
  uint8_t port;
  const bool inverted;
  bool wasPressedLast;

  virtual bool currentlyPressed();
};
} // namespace okapi

#endif
