/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/controller.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "okapi/impl/device/controllerUtil.hpp"

namespace okapi {
std::array<ControllerButton *, 12> Controller::buttonArray;

Controller::Controller(const ControllerId iid)
  : m_id(iid), controller(ControllerUtil::idToProsEnum(iid)) {
  std::fill(buttonArray.begin(), buttonArray.end(), nullptr);
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

ControllerButton &Controller::operator[](const ControllerDigital ibtn) {
  const auto index = toUnderlyingType(ibtn) - toUnderlyingType(ControllerDigital::L1);

  if (buttonArray[index] == nullptr) {
    buttonArray[index] = new ControllerButton(m_id, ibtn);
  }

  return *buttonArray[index];
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

std::int32_t Controller::rumble(std::string irumblePattern) {
  return controller.rumble(irumblePattern.c_str());
}

std::int32_t Controller::getBatteryCapacity() {
  return controller.get_battery_capacity();
}

std::int32_t Controller::getBatteryLevel() {
  return controller.get_battery_level();
}
} // namespace okapi
