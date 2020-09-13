/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncVelPidController.hpp"
#include "okapi/api/util/mathUtil.hpp"

namespace okapi {
AsyncVelPIDController::AsyncVelPIDController(
  const std::shared_ptr<ControllerInput<double>> &iinput,
  const std::shared_ptr<ControllerOutput<double>> &ioutput,
  const TimeUtil &itimeUtil,
  const double ikP,
  const double ikD,
  const double ikF,
  const double ikSF,
  std::unique_ptr<VelMath> ivelMath,
  const double iratio,
  std::unique_ptr<Filter> iderivativeFilter,
  const std::shared_ptr<Logger> &ilogger)
  : AsyncWrapper<double, double>(
      iinput,
      ioutput,
      std::make_shared<IterativeVelPIDController>(ikP,
                                                  ikD,
                                                  ikF,
                                                  ikSF,
                                                  std::move(ivelMath),
                                                  itimeUtil,
                                                  std::move(iderivativeFilter)),
      itimeUtil.getRateSupplier(),
      iratio,
      ilogger),
    internalController(std::static_pointer_cast<IterativeVelPIDController>(controller)) {
}

void AsyncVelPIDController::setGains(const IterativeVelPIDController::Gains &igains) {
  internalController->setGains(igains);
}

IterativeVelPIDController::Gains AsyncVelPIDController::getGains() const {
  return internalController->getGains();
}
} // namespace okapi
