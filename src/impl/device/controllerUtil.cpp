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
  // TODO: Implement this
}
} // namespace okapi
