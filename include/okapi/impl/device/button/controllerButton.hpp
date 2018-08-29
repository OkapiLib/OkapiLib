/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CONTROLLERBUTTON_HPP_
#define _OKAPI_CONTROLLERBUTTON_HPP_

#include "api.h"
#include "okapi/api/device/button/buttonBase.hpp"
#include "okapi/impl/device/controllerUtil.hpp"

namespace okapi {
class ControllerButton : public ButtonBase {
  public:
  ControllerButton(ControllerDigital ibtn, bool iinverted = false);

  ControllerButton(ControllerId icontroller, ControllerDigital ibtn, bool iinverted = false);

  protected:
  pros::Controller controller;
  const ControllerDigital btn;

  virtual bool currentlyPressed() override;
};
} // namespace okapi

#endif
