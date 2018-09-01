/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/button/controllerButton.hpp"

namespace okapi {
ControllerButton::ControllerButton(const ControllerDigital ibtn, const bool iinverted)
  : ButtonBase(iinverted),
    controller(ControllerUtil::idToProsEnum(ControllerId::master)),
    btn(ibtn) {
}

ControllerButton::ControllerButton(const ControllerId icontroller,
                                   const ControllerDigital ibtn,
                                   const bool iinverted)
  : ButtonBase(iinverted), controller(ControllerUtil::idToProsEnum(icontroller)), btn(ibtn) {
}

bool ControllerButton::currentlyPressed() {
  const bool pressed = controller.get_digital(ControllerUtil::digitalToProsEnum(btn)) != 0;
  return inverted ? !pressed : pressed;
}
} // namespace okapi
