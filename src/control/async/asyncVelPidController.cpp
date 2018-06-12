/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/control/async/asyncVelPidController.hpp"

namespace okapi {
AsyncVelPIDControllerArgs::AsyncVelPIDControllerArgs(std::shared_ptr<ControllerInput> iinput,
                                                     std::shared_ptr<ControllerOutput> ioutput,
                                                     const IterativeVelPIDControllerArgs &iparams)
  : input(iinput), output(ioutput), params(iparams) {
}

AsyncVelPIDController::AsyncVelPIDController(std::shared_ptr<ControllerInput> iinput,
                                             std::shared_ptr<ControllerOutput> ioutput,
                                             const IterativeVelPIDControllerArgs &iparams)
  : AsyncWrapper(iinput, ioutput, std::make_unique<IterativeVelPIDController>(iparams)) {
}

AsyncVelPIDController::AsyncVelPIDController(std::shared_ptr<ControllerInput> iinput,
                                             std::shared_ptr<ControllerOutput> ioutput,
                                             const double ikP, const double ikD, const double ikF)
  : AsyncWrapper(iinput, ioutput, std::make_unique<IterativeVelPIDController>(ikP, ikD, ikF)) {
}
} // namespace okapi
