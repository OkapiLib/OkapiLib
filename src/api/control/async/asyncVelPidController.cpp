/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncVelPidController.hpp"
#include "okapi/api/util/mathUtil.hpp"

namespace okapi {
AsyncVelPIDController::AsyncVelPIDController(
  std::shared_ptr<ControllerInput> iinput, std::shared_ptr<ControllerOutput> ioutput,
  const Supplier<std::unique_ptr<AbstractRate>> &irateSupplier,
  std::unique_ptr<AbstractTimer> itimer,
  const Supplier<std::unique_ptr<SettledUtil>> &isettledUtilSupplier, const double ikP,
  const double ikD, const double ikF, std::unique_ptr<VelMath> ivelMath)
  : AsyncWrapper(iinput, ioutput,
                 std::make_unique<IterativeVelPIDController>(ikP, ikD, ikF, std::move(ivelMath),
                                                             std::move(itimer),
                                                             isettledUtilSupplier.get()),
                 irateSupplier, isettledUtilSupplier.get()) {
}
} // namespace okapi
