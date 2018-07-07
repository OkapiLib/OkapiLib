/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/control/async/asyncPosPidController.hpp"
#include "okapi/impl/util/timer.hpp"

namespace okapi {
AsyncPosPIDControllerArgs::AsyncPosPIDControllerArgs(std::shared_ptr<ControllerInput> iinput,
                                                     std::shared_ptr<ControllerOutput> ioutput,
                                                     const IterativePosPIDControllerArgs &iparams)
  : input(iinput), output(ioutput), params(iparams) {
}

AsyncPosPIDController::AsyncPosPIDController(std::shared_ptr<ControllerInput> iinput,
                                             std::shared_ptr<ControllerOutput> ioutput,
                                             const double ikP, const double ikI, const double ikD,
                                             const double ikBias)
  : AsyncWrapper(iinput, ioutput,
                 std::make_unique<IterativePosPIDController>(
                   ikP, ikI, ikD, ikBias, std::make_unique<Timer>(),
                   std::make_unique<SettledUtil>(std::make_unique<Timer>()))) {
}

AsyncPosPIDController::AsyncPosPIDController(std::shared_ptr<ControllerInput> iinput,
                                             std::shared_ptr<ControllerOutput> ioutput,
                                             const IterativePosPIDControllerArgs &iparams)
  : AsyncWrapper(iinput, ioutput,
                 std::make_unique<IterativePosPIDController>(
                   iparams, std::make_unique<Timer>(),
                   std::make_unique<SettledUtil>(std::make_unique<Timer>()))) {
}
} // namespace okapi
