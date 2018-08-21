/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/controllerUtil.hpp"

namespace okapi {
controller_id_e_t ControllerUtil::idToProsEnum(ControllerId in) {
  switch (in) {
  case ControllerId::master:
    return E_CONTROLLER_MASTER;
  case ControllerId::partner:
    return E_CONTROLLER_PARTNER;
  }
}

controller_analog_e_t ControllerUtil::analogToProsEnum(ControllerAnalog in) {
  switch (in) {
  case ControllerAnalog::leftX:
    return E_CONTROLLER_ANALOG_LEFT_X;
  case ControllerAnalog::leftY:
    return E_CONTROLLER_ANALOG_LEFT_Y;
  case ControllerAnalog::rightX:
    return E_CONTROLLER_ANALOG_RIGHT_X;
  case ControllerAnalog::rightY:
    return E_CONTROLLER_ANALOG_RIGHT_Y;
  }
}

controller_digital_e_t ControllerUtil::digitalToProsEnum(ControllerDigital in) {
  switch (in) {
  case ControllerDigital::L1:
    return E_CONTROLLER_DIGITAL_L1;
  case ControllerDigital::L2:
    return E_CONTROLLER_DIGITAL_L2;
  case ControllerDigital::R1:
    return E_CONTROLLER_DIGITAL_R1;
  case ControllerDigital::R2:
    return E_CONTROLLER_DIGITAL_R2;
  case ControllerDigital::up:
    return E_CONTROLLER_DIGITAL_UP;
  case ControllerDigital::down:
    return E_CONTROLLER_DIGITAL_DOWN;
  case ControllerDigital::left:
    return E_CONTROLLER_DIGITAL_LEFT;
  case ControllerDigital::right:
    return E_CONTROLLER_DIGITAL_RIGHT;
  case ControllerDigital::X:
    return E_CONTROLLER_DIGITAL_X;
  case ControllerDigital::B:
    return E_CONTROLLER_DIGITAL_B;
  case ControllerDigital::Y:
    return E_CONTROLLER_DIGITAL_Y;
  case ControllerDigital::A:
    return E_CONTROLLER_DIGITAL_A;
  }
}
} // namespace okapi
