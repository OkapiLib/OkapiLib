/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/device/controller.hpp"

namespace okapi {
Controller::Controller(const controller_id_e_t iid) : controller(iid) {
}

Controller::~Controller() = default;

bool Controller::isConnected() {
  const int32_t state = controller.is_connected();
  return state == 1 || state == 2;
}

int32_t Controller::getConnectionState() {
  return controller.is_connected();
}

float Controller::getAnalog(const controller_analog_e_t ichannel) {
  return controller.get_analog(ichannel) / 127.0;
}

bool Controller::getDigital(const controller_digital_e_t ibutton) {
  return controller.get_digital(ibutton) == 1;
}
} // namespace okapi
