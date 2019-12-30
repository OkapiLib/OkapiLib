/*
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncPosPidController.hpp"

namespace okapi {
AsyncPosPIDController::AsyncPosPIDController(
  const std::shared_ptr<ControllerInput<double>> &iinput,
  const std::shared_ptr<ControllerOutput<double>> &ioutput,
  const TimeUtil &itimeUtil,
  const double ikP,
  const double ikI,
  const double ikD,
  const double ikBias,
  const double iratio,
  std::unique_ptr<Filter> iderivativeFilter,
  const std::shared_ptr<Logger> &ilogger)
  : AsyncPosPIDController(std::make_shared<OffsetableControllerInput>(iinput),
                          ioutput,
                          itimeUtil,
                          ikP,
                          ikI,
                          ikD,
                          ikBias,
                          iratio,
                          std::move(iderivativeFilter),
                          ilogger) {
}

AsyncPosPIDController::AsyncPosPIDController(
  const std::shared_ptr<OffsetableControllerInput> &iinput,
  const std::shared_ptr<ControllerOutput<double>> &ioutput,
  const TimeUtil &itimeUtil,
  const double ikP,
  const double ikI,
  const double ikD,
  const double ikBias,
  const double iratio,
  std::unique_ptr<Filter> iderivativeFilter,
  const std::shared_ptr<Logger> &ilogger)
  : AsyncWrapper<double, double>(
      iinput,
      ioutput,
      std::make_unique<IterativePosPIDController>(ikP,
                                                  ikI,
                                                  ikD,
                                                  ikBias,
                                                  itimeUtil,
                                                  std::move(iderivativeFilter)),
      itimeUtil.getRateSupplier(),
      iratio,
      ilogger),
    offsettableInput(iinput) {
}

void AsyncPosPIDController::tarePosition() {
  offsettableInput->tarePosition();
}
} // namespace okapi
