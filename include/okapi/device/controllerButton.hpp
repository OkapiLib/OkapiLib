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

    virtual ~ControllerButton() = default;

    bool isPressed();

    bool edge();

    bool risingEdge();

    bool fallingEdge();

  private:
    pros::Controller controller;
    controller_digital_e_t btn;
    const bool inverted;
    bool wasPressedLast;

    bool currentlyPressed();
  };
}

#endif
