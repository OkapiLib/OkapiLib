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

namespace okapi {
class ControllerButton : public ButtonBase {
  public:
  ControllerButton(const controller_digital_e_t ibtn, const bool iinverted = false);

  ControllerButton(const controller_id_e_t icontroller,
                   const controller_digital_e_t ibtn,
                   const bool iinverted = false);

  protected:
  pros::Controller controller;
  const controller_digital_e_t btn;

  virtual bool currentlyPressed() override;
};
} // namespace okapi

#endif
