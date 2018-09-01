/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/control/iterative/iterativeControllerFactory.hpp"
#include "okapi/impl/filter/velMathFactory.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"

namespace okapi {
IterativePosPIDController
IterativeControllerFactory::posPID(const double ikP,
                                   const double ikI,
                                   const double ikD,
                                   const double ikBias,
                                   std::unique_ptr<Filter> iderivativeFilter) {
  return IterativePosPIDController(
    ikP, ikI, ikD, ikBias, TimeUtilFactory::create(), std::move(iderivativeFilter));
}

IterativeVelPIDController
IterativeControllerFactory::velPID(const double ikP,
                                   const double ikD,
                                   const double ikF,
                                   const double ikSF,
                                   const VelMathArgs &iparams,
                                   std::unique_ptr<Filter> iderivativeFilter) {
  return IterativeVelPIDController(ikP,
                                   ikD,
                                   ikF,
                                   ikSF,
                                   VelMathFactory::createPtr(iparams),
                                   TimeUtilFactory::create(),
                                   std::move(iderivativeFilter));
}

IterativeMotorVelocityController
IterativeControllerFactory::motorVelocity(Motor imotor,
                                          const double ikP,
                                          const double ikD,
                                          const double ikF,
                                          const double ikSF,
                                          const VelMathArgs &iparams) {
  return IterativeMotorVelocityController(
    std::make_shared<Motor>(imotor),
    std::make_shared<IterativeVelPIDController>(velPID(ikP, ikD, ikF, ikSF, iparams)));
}

IterativeMotorVelocityController
IterativeControllerFactory::motorVelocity(MotorGroup imotor,
                                          const double ikP,
                                          const double ikD,
                                          const double ikF,
                                          const double ikSF,
                                          const VelMathArgs &iparams) {
  return IterativeMotorVelocityController(
    std::make_shared<MotorGroup>(imotor),
    std::make_shared<IterativeVelPIDController>(velPID(ikP, ikD, ikF, ikSF, iparams)));
}

IterativeMotorVelocityController IterativeControllerFactory::motorVelocity(
  Motor imotor,
  std::shared_ptr<IterativeVelocityController<double, double>> icontroller) {
  return IterativeMotorVelocityController(std::make_shared<Motor>(imotor), icontroller);
}

IterativeMotorVelocityController IterativeControllerFactory::motorVelocity(
  MotorGroup imotor,
  std::shared_ptr<IterativeVelocityController<double, double>> icontroller) {
  return IterativeMotorVelocityController(std::make_shared<MotorGroup>(imotor), icontroller);
}
} // namespace okapi
