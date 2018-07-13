/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/util/supplier.hpp"
#include "okapi/impl/control/util/pidTunerFactory.hpp"
#include "okapi/impl/util/timer.hpp"
#include "okapi/impl/util/rate.hpp"
#include "okapi/impl/control/util/settledUtilFactory.hpp"
#include "okapi/impl/control/iterative/iterativeControllerFactory.hpp"

namespace okapi {
PIDTuner PIDTunerFactory::create(std::shared_ptr<ControllerInput> iinput,
         std::shared_ptr<ControllerOutput> ioutput,
         QTime itimeout, std::int32_t igoal, double ikPMin, double ikPMax, double ikIMin,
         double ikIMax, double ikDMin, double ikDMax, std::int32_t inumIterations,
         std::int32_t inumParticles, double ikSettle, double ikITAE) {
  return PIDTuner(iinput, ioutput, Supplier<std::unique_ptr<AbstractTimer>>([]() { return std::make_unique<Timer>(); }),
                  Supplier<std::unique_ptr<SettledUtil>>([]() { return SettledUtilFactory::createPtr(); }),
                  Supplier<std::unique_ptr<AbstractRate>>([]() { return std::make_unique<Rate>(); }),
                  itimeout, igoal, ikPMin, ikPMax,
                  ikIMin, ikIMax, ikDMin, ikDMax, inumIterations, inumParticles,
                  ikSettle, ikITAE);
}

std::unique_ptr<PIDTuner> PIDTunerFactory::createPtr(std::shared_ptr<ControllerInput> iinput,
         std::shared_ptr<ControllerOutput> ioutput,
         QTime itimeout, std::int32_t igoal, double ikPMin, double ikPMax, double ikIMin,
         double ikIMax, double ikDMin, double ikDMax, std::int32_t inumIterations,
         std::int32_t inumParticles, double ikSettle, double ikITAE) {
  return std::make_unique<PIDTuner>(iinput, ioutput, Supplier<std::unique_ptr<AbstractTimer>>([]() { return std::make_unique<Timer>(); }),
                  Supplier<std::unique_ptr<SettledUtil>>([]() { return SettledUtilFactory::createPtr(); }),
                  Supplier<std::unique_ptr<AbstractRate>>([]() { return std::make_unique<Rate>(); }),
                  itimeout, igoal, ikPMin, ikPMax,
                  ikIMin, ikIMax, ikDMin, ikDMax, inumIterations, inumParticles,
                  ikSettle, ikITAE);
}
} // namespace okapi
