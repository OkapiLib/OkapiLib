/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/controllerUtil.hpp"

namespace okapi {
pros::controller_id_e_t ControllerUtil::idToProsEnum(ControllerId in) {
  switch (in) {
  case ControllerId::master:
    return pros::E_CONTROLLER_MASTER;
  case ControllerId::partner:
    return pros::E_CONTROLLER_PARTNER;
  }
}

pros::controller_analog_e_t ControllerUtil::analogToProsEnum(ControllerAnalog in) {
  switch (in) {
  case ControllerAnalog::leftX:
    return pros::E_CONTROLLER_ANALOG_LEFT_X;
  case ControllerAnalog::leftY:
    return pros::E_CONTROLLER_ANALOG_LEFT_Y;
  case ControllerAnalog::rightX:
    return pros::E_CONTROLLER_ANALOG_RIGHT_X;
  case ControllerAnalog::rightY:
    return pros::E_CONTROLLER_ANALOG_RIGHT_Y;
  }
}

pros::controller_digital_e_t ControllerUtil::digitalToProsEnum(ControllerDigital in) {
  switch (in) {
  case ControllerDigital::L1:
    return pros::E_CONTROLLER_DIGITAL_L1;
  case ControllerDigital::L2:
    return pros::E_CONTROLLER_DIGITAL_L2;
  case ControllerDigital::R1:
    return pros::E_CONTROLLER_DIGITAL_R1;
  case ControllerDigital::R2:
    return pros::E_CONTROLLER_DIGITAL_R2;
  case ControllerDigital::up:
    return pros::E_CONTROLLER_DIGITAL_UP;
  case ControllerDigital::down:
    return pros::E_CONTROLLER_DIGITAL_DOWN;
  case ControllerDigital::left:
    return pros::E_CONTROLLER_DIGITAL_LEFT;
  case ControllerDigital::right:
    return pros::E_CONTROLLER_DIGITAL_RIGHT;
  case ControllerDigital::X:
    return pros::E_CONTROLLER_DIGITAL_X;
  case ControllerDigital::B:
    return pros::E_CONTROLLER_DIGITAL_B;
  case ControllerDigital::Y:
    return pros::E_CONTROLLER_DIGITAL_Y;
  case ControllerDigital::A:
    return pros::E_CONTROLLER_DIGITAL_A;
  }
}
} // namespace okapi
