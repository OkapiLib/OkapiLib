/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncPosPidController.hpp"

namespace okapi {
AsyncPosPIDController::AsyncPosPIDController(std::shared_ptr<ControllerInput> iinput,
                                             std::shared_ptr<ControllerOutput> ioutput,
                                             std::unique_ptr<AbstractRate> irate,
                                             std::unique_ptr<AbstractTimer> itimer,
                                             std::unique_ptr<SettledUtil> isettledUtil,
                                             const double ikP, const double ikI, const double ikD,
                                             const double ikBias)
  : AsyncWrapper(iinput, ioutput,
                 std::make_unique<IterativePosPIDController>(
                   ikP, ikI, ikD, ikBias, std::move(itimer), std::move(isettledUtil)),
                 std::move(irate)) {
}
} // namespace okapi
