/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/control/util/pidTunerFactory.hpp"
#include "okapi/impl/control/iterative/iterativeControllerFactory.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"

namespace okapi {
PIDTuner PIDTunerFactory::create(const std::shared_ptr<ControllerInput<double>> &iinput,
                                 const std::shared_ptr<ControllerOutput<double>> &ioutput,
                                 QTime itimeout,
                                 std::int32_t igoal,
                                 double ikPMin,
                                 double ikPMax,
                                 double ikIMin,
                                 double ikIMax,
                                 double ikDMin,
                                 double ikDMax,
                                 std::int32_t inumIterations,
                                 std::int32_t inumParticles,
                                 double ikSettle,
                                 double ikITAE,
                                 const std::shared_ptr<Logger> &ilogger) {
  return PIDTuner(iinput,
                  ioutput,
                  TimeUtilFactory::createDefault(),
                  itimeout,
                  igoal,
                  ikPMin,
                  ikPMax,
                  ikIMin,
                  ikIMax,
                  ikDMin,
                  ikDMax,
                  inumIterations,
                  inumParticles,
                  ikSettle,
                  ikITAE,
                  ilogger);
}

std::unique_ptr<PIDTuner>
PIDTunerFactory::createPtr(const std::shared_ptr<ControllerInput<double>> &iinput,
                           const std::shared_ptr<ControllerOutput<double>> &ioutput,
                           QTime itimeout,
                           std::int32_t igoal,
                           double ikPMin,
                           double ikPMax,
                           double ikIMin,
                           double ikIMax,
                           double ikDMin,
                           double ikDMax,
                           std::int32_t inumIterations,
                           std::int32_t inumParticles,
                           double ikSettle,
                           double ikITAE,
                           const std::shared_ptr<Logger> &ilogger) {
  return std::make_unique<PIDTuner>(iinput,
                                    ioutput,
                                    TimeUtilFactory::createDefault(),
                                    itimeout,
                                    igoal,
                                    ikPMin,
                                    ikPMax,
                                    ikIMin,
                                    ikIMax,
                                    ikDMin,
                                    ikDMax,
                                    inumIterations,
                                    inumParticles,
                                    ikSettle,
                                    ikITAE,
                                    ilogger);
}
} // namespace okapi
