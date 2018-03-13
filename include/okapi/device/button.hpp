/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_BUTTON_HPP_
#define _OKAPI_BUTTON_HPP_

#include "api.h"

namespace okapi {
class Button {
  public:
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
};
} // namespace okapi

#endif
