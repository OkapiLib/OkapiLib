/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/control/async/asyncVelPidController.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "okapi/impl/control/util/settledUtilFactory.hpp"
#include "okapi/impl/util/timer.hpp"

namespace okapi {
AsyncVelPIDController::AsyncVelPIDController(std::shared_ptr<ControllerInput> iinput,
                                             std::shared_ptr<ControllerOutput> ioutput,
                                             const double ikP, const double ikD, const double ikF,
                                             std::unique_ptr<VelMath> ivelMath)
  : AsyncWrapper(iinput, ioutput,
                 std::make_unique<IterativeVelPIDController>(ikP, ikD, ikF, std::move(ivelMath),
                                                             std::make_unique<Timer>(),
                                                             SettledUtilFactory::createPtr())) {
}
} // namespace okapi
