/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/offsettableControllerInput.hpp"
#include "okapi/api/util/mathUtil.hpp"

namespace okapi {
OffsettableControllerInput::OffsettableControllerInput(
  const std::shared_ptr<ControllerInput<double>> &iinput)
  : input(iinput) {
}

OffsettableControllerInput::~OffsettableControllerInput() = default;

double OffsettableControllerInput::controllerGet() {
  return input->controllerGet() - offset;
}

void OffsettableControllerInput::tarePosition() {
  const auto reading = input->controllerGet();
  if (reading != OKAPI_PROS_ERR) {
    offset = reading;
  }
}
} // namespace okapi
