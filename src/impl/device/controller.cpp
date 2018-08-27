/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/controller.hpp"
#include "okapi/impl/device/controllerUtil.hpp"

namespace okapi {
Controller::Controller(const ControllerId iid)
  : m_id(iid), controller(ControllerUtil::idToProsEnum(iid)) {
}

Controller::~Controller() = default;

bool Controller::isConnected() {
  const std::int32_t state = controller.is_connected();
  return state == 1 || state == 2;
}

std::int32_t Controller::getConnectionState() {
  return controller.is_connected();
}

float Controller::getAnalog(const ControllerAnalog ichannel) {
  const auto val = controller.get_analog(ControllerUtil::analogToProsEnum(ichannel));
  if (val == PROS_ERR) {
    return 0;
  }

  return static_cast<float>(val) / static_cast<float>(127);
}

bool Controller::getDigital(const ControllerDigital ibutton) {
  return controller.get_digital(ControllerUtil::digitalToProsEnum(ibutton)) == 1;
}

ControllerButton Controller::operator[](const ControllerDigital ibtn) {
  return ControllerButton(m_id, ibtn);
}

std::int32_t Controller::setText(std::uint8_t iline, std::uint8_t icol, std::string itext) {
  return controller.set_text(iline, icol, itext.c_str());
}

std::int32_t Controller::clear() {
  return controller.clear();
}

std::int32_t Controller::clearLine(std::uint8_t iline) {
  return controller.clear_line(iline);
}

std::int32_t Controller::getBatteryCapacity() {
  return controller.get_battery_capacity();
}

std::int32_t Controller::getBatteryLevel() {
  return controller.get_battery_level();
}
} // namespace okapi
