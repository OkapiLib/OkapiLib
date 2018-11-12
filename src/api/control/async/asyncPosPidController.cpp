/**
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
  std::unique_ptr<Filter> iderivativeFilter)
  : AsyncWrapper<double, double>(
      iinput,
      ioutput,
      std::make_unique<IterativePosPIDController>(ikP,
                                                  ikI,
                                                  ikD,
                                                  ikBias,
                                                  itimeUtil,
                                                  std::move(iderivativeFilter)),
      itimeUtil.getRateSupplier()) {
}
} // namespace okapi
