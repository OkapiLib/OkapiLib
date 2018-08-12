/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/button/controllerButton.hpp"

namespace okapi {
ControllerButton::ControllerButton(const controller_digital_e_t ibtn, const bool iinverted)
  : ButtonBase(iinverted), controller(E_CONTROLLER_MASTER), btn(ibtn) {
}

ControllerButton::ControllerButton(const controller_id_e_t icontroller,
                                   const controller_digital_e_t ibtn,
                                   const bool iinverted)
  : ButtonBase(iinverted), controller(icontroller), btn(ibtn) {
}

bool ControllerButton::currentlyPressed() {
  const bool pressed = controller.get_digital(btn) != 0;
  return inverted ? !pressed : pressed;
}
} // namespace okapi
