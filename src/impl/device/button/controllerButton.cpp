/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/button/controllerButton.hpp"

namespace okapi {
ControllerButton::ControllerButton(const ControllerDigital ibtn, const bool iinverted)
  : ControllerButton(ControllerId::master, ibtn, iinverted) {
}

ControllerButton::ControllerButton(const ControllerId icontroller,
                                   const ControllerDigital ibtn,
                                   const bool iinverted)
  : ButtonBase(iinverted),
    id(ControllerUtil::idToProsEnum(icontroller)),
    btn(ControllerUtil::digitalToProsEnum(ibtn)) {
}

bool ControllerButton::currentlyPressed() {
  const bool pressed = pros::c::controller_get_digital(id, btn) != 0;
  return inverted == !pressed;
}
} // namespace okapi
