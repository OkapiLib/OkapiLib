/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/offsettableControllerInput.hpp"
#include "okapi/api/util/mathUtil.hpp"

namespace okapi {
OffsetableControllerInput::OffsetableControllerInput(
  const std::shared_ptr<ControllerInput<double>> &iinput)
  : input(iinput) {
}

OffsetableControllerInput::~OffsetableControllerInput() = default;

double OffsetableControllerInput::controllerGet() {
  return input->controllerGet() - offset;
}

void OffsetableControllerInput::tarePosition() {
  const auto reading = input->controllerGet();
  if (reading != OKAPI_PROS_ERR) {
    offset = reading;
  }
}
} // namespace okapi
