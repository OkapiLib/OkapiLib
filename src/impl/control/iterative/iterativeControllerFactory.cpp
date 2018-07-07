/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/control/iterative/iterativeControllerFactory.hpp"
#include "okapi/api/control/util/settledUtil.hpp"
#include "okapi/impl/util/timer.hpp"

namespace okapi {
IterativePosPIDController IterativeControllerFactory::createPosPID(const double ikP,
                                                                   const double ikI,
                                                                   const double ikD,
                                                                   const double ikBias) {
  return IterativePosPIDController(ikP, ikI, ikD, ikBias, std::make_unique<Timer>(),
                                   std::make_unique<SettledUtil>(std::make_unique<Timer>()));
}

IterativeVelPIDController IterativeControllerFactory::createVelPID(const double ikP,
                                                                   const double ikD,
                                                                   const double ikF,
                                                                   const VelMathArgs &iparams) {
  return IterativeVelPIDController(IterativeVelPIDControllerArgs(ikP, ikD, ikF, iparams),
                                   std::make_unique<Timer>(),
                                   std::make_unique<SettledUtil>(std::make_unique<Timer>()));
}
} // namespace okapi
