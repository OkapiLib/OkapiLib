/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_CONTROLLERBUTTON_HPP_
#define _OKAPI_CONTROLLERBUTTON_HPP_

#include "okapi/device/button.hpp"

namespace okapi {
class ControllerButton : public Button {
  public:
  ControllerButton(controller_id_e_t icontroller, controller_digital_e_t ibtn,
                   const bool iinverted = false);

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
  pros::Controller controller;
  controller_digital_e_t btn;
  const bool inverted;
  bool wasPressedLast;

  bool currentlyPressed();
};
} // namespace okapi

#endif
