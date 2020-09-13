/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/controller.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "okapi/impl/device/controllerUtil.hpp"

namespace okapi {
Controller::Controller(const ControllerId iid)
  : okapiId(iid), prosId(ControllerUtil::idToProsEnum(iid)) {
}

Controller::~Controller() = default;

bool Controller::isConnected() {
  const std::int32_t state = pros::c::controller_is_connected(prosId);
  return state == 1 || state == 2;
}

float Controller::getAnalog(const ControllerAnalog ichannel) {
  const auto val =
    pros::c::controller_get_analog(prosId, ControllerUtil::analogToProsEnum(ichannel));

  if (val == PROS_ERR) {
    return 0;
  }

  return static_cast<float>(val) / static_cast<float>(127);
}

bool Controller::getDigital(const ControllerDigital ibutton) {
  return pros::c::controller_get_digital(prosId, ControllerUtil::digitalToProsEnum(ibutton)) == 1;
}

ControllerButton &Controller::operator[](const ControllerDigital ibtn) {
  const auto index = toUnderlyingType(ibtn) - toUnderlyingType(ControllerDigital::L1);

  if (buttonArray[index] == nullptr) {
    buttonArray[index] = new ControllerButton(okapiId, ibtn);
  }

  return *buttonArray[index];
}

std::int32_t Controller::setText(std::uint8_t iline, std::uint8_t icol, std::string itext) {
  return pros::c::controller_set_text(prosId, iline, icol, itext.c_str());
}

std::int32_t Controller::clear() {
  return pros::c::controller_clear(prosId);
}

std::int32_t Controller::clearLine(std::uint8_t iline) {
  return pros::c::controller_clear_line(prosId, iline);
}

std::int32_t Controller::rumble(std::string irumblePattern) {
  return pros::c::controller_rumble(prosId, irumblePattern.c_str());
}

std::int32_t Controller::getBatteryCapacity() {
  return pros::c::controller_get_battery_capacity(prosId);
}

std::int32_t Controller::getBatteryLevel() {
  return pros::c::controller_get_battery_level(prosId);
}
} // namespace okapi
