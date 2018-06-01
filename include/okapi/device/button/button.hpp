/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_BUTTON_HPP_
#define _OKAPI_BUTTON_HPP_

#include "api.h"

namespace okapi {
class AbstractButton {
  public:
  AbstractButton(const bool iinverted = false);

  virtual ~AbstractButton();

  /**
   * Return whether the button is currently pressed.
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
  const bool inverted;
  bool wasPressedLast = false;

  virtual bool currentlyPressed() = 0;
};
} // namespace okapi

#endif
